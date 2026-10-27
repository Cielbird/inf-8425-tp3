from dataclasses import dataclass

from drone import Drone
from gnss import ModuleGNSS
from imu import ModuleIMU
from util import PID3, Vector3


@dataclass
class TaskConfig:
    """
    Paramètres d'une tâche périodique
    """
    frequence: float # Hz
    processing_dt: float # durée du calcul (s)
    pid: tuple[float, float, float] # (p, i, d)
    integrateur_max: float # intégrateur du PID limité à ±integrateur_max

    def make_pid(self) -> PID3:
        return PID3(*self.pid, 1.0 / self.frequence, -self.integrateur_max, self.integrateur_max)


class Task:
    """
    Tâche périodique de l'RTOS.
    waiting -> (activation) -> ready -> (le MCU lui donne le CPU) -> calculating -> waiting
    """

    def __init__(self, period: float, processing_dt: float):
        self.period = period
        # read and writing times are negligible
        self.processing_dt = processing_dt # calculating time in seconds
        self.status = "waiting" # in {"waiting", "ready", "calculating"}
        self.next_release = 0.0 # prochaine activation périodique
        self.start_t = 0.0 # début du calcul en cours
        self.exec_log: list[tuple[float, float]] = [] # (début, durée) de chaque calcul

    def next_event_t(self) -> float:
        """
        Get the time (seconds) of the next event
        """
        match self.status:
            case "waiting":
                return self.next_release
            case "ready":
                # attend que le MCU lui donne le CPU, pas d'événement propre
                return float("inf")
            case "calculating":
                return self.start_t + self.processing_dt
            case _:
                raise TypeError("Invalid status")

    def start(self, t: float):
        """
        Le MCU donne le CPU à la tâche
        """
        assert self.status == "ready"
        self.start_t = t
        self.status = "calculating"

    def step(self):
        """
        Run sim until next event
        """
        t = self.next_event_t()
        match self.status:
            case "waiting":
                self.next_release += self.period
                self.status = "ready"

            case "calculating":
                self.exec_log.append((self.start_t, t - self.start_t))
                self.compute()
                if self.next_release <= t:
                    # échéance manquée: déjà réactivée pendant le calcul
                    self.next_release += self.period
                    self.status = "ready"
                else:
                    self.status = "waiting"

            case _:
                raise TypeError("Invalid status")

    def compute(self):
        """
        Lecture des entrées, calcul et écriture des sorties, à la fin du calcul
        """
        raise NotImplementedError


class NavigationTask(Task):
    """
    Lire la position (GNSS via UART), calculer la vitesse nécessaire, et écrire en mémoire partagée.
    """

    def __init__(self, gnss: ModuleGNSS, imu: ModuleIMU, flight_plan: list[Vector3], config: TaskConfig):
        super().__init__(period=1.0 / config.frequence, processing_dt=config.processing_dt)
        self.gnss = gnss
        self.imu = imu

        # task variables
        self.position = Vector3(0.0, 0.0, 0.0) # position estimée x, y, z (m)
        self.velocity = Vector3(0.0, 0.0, 0.0) # "shared" memory

        # navigation à l'estime (dead reckoning)
        self.vitesse_estimee = Vector3(0.0, 0.0, 0.0)
        self.position_gnss = Vector3(0.0, 0.0, 0.0) # dernière position GNSS lue
        self.t_lecture = 0.0 # instant de la dernière lecture des capteurs
        self.fix = "3D" # in {"3D", "2D", "aucun"}
        self.event_log: list[tuple[float, str]] = [] # (t, nouveau fix) à chaque changement détecté
        self.nb_mises_a_jour = 0 # incrémenté à chaque nouvelle estimation, "shared" memory

        self.next_checkpoint_idx = 1
        self.flight_plan = flight_plan
        self.pid = config.make_pid()
        self.arrival_dist = 0.3

    def compute(self):
        # read from GNSS and IMU
        satelites = self.gnss.get_satelites()
        position_gnss = self.gnss.get_position()
        delta_v = self.imu.get_delta_v()
        # temps écoulé depuis la dernière lecture, mesuré par l'horloge du MCU
        t = self.start_t + self.processing_dt
        dt = t - self.t_lecture
        self.t_lecture = t

        # détection de la perte / du retour du fix
        fix = "3D" if satelites >= 4 else "2D" if satelites == 3 else "aucun"
        fix_precedent = self.fix
        if fix != self.fix:
            self.event_log.append((self.start_t, fix))
            self.fix = fix

        # estime: intégration du delta-v de l'IMU depuis la dernière position
        # (vitesse moyenne sur l'intervalle: exacte si l'accélération y est constante)
        estime = self.position + (self.vitesse_estimee + delta_v / 2) * dt
        self.vitesse_estimee += delta_v

        match fix:
            case "3D":
                if fix_precedent == "3D":
                    # recale la vitesse sur deux positions GNSS consécutives: leur différence donne
                    # la vitesse moyenne, en milieu d'intervalle; + delta_v / 2 pour la vitesse actuelle
                    self.vitesse_estimee = (position_gnss - self.position_gnss) / dt + delta_v / 2
                self.position = position_gnss
            case "2D":
                # x, y du GNSS, altitude figée par le récepteur: z à l'estime
                self.position = Vector3(position_gnss.x, position_gnss.y, estime.z)
            case _:
                self.position = estime
        self.position_gnss = position_gnss

        # write to shared mem
        self.velocity = self.calculate_vel()
        self.nb_mises_a_jour += 1

    def calculate_vel(self) -> Vector3:
        target = self.flight_plan[self.next_checkpoint_idx]
        if (target - self.position).magnitude() < self.arrival_dist:
            if self.next_checkpoint_idx < len(self.flight_plan) - 1:
                self.next_checkpoint_idx += 1
        return self.pid(self.position, target)


class ControleMoteursTask(Task):
    """
    Lire la vitesse (IMU via I2C), calculer la force nécessaire, et envoyer commande aux
    moteurs (PWM).
    """

    def __init__(self, drone: Drone, imu: ModuleIMU, nav_task: NavigationTask, config: TaskConfig):
        super().__init__(period=1.0 / config.frequence, processing_dt=config.processing_dt)
        self.drone = drone
        self.imu = imu
        self.nav_task = nav_task

        # task variables
        self.vitesse_imu = Vector3(0.0, 0.0, 0.0) # intégrateur après le capteur IMU, avant PID
        self.nav_vue = 0 # dernière estimation de la navigation utilisée
        self.pid = config.make_pid() # sortie en force (N)

    def compute(self):
        # read from shared mem
        vel = self.nav_task.velocity

        accel_imu = self.imu.get_accel()
        if self.nav_task.nb_mises_a_jour != self.nav_vue:
            # nouvelle estimation de la navigation (recalée sur le GNSS si fix 3D):
            # remplace l'intégration IMU pour éviter la dérive
            self.nav_vue = self.nav_task.nb_mises_a_jour
            self.vitesse_imu = self.nav_task.vitesse_estimee.copy()
        else:
            self.vitesse_imu += accel_imu * self.period
        force = self.pid(self.vitesse_imu, vel)

        # write to motors
        self.drone.set_force(force)


class Microcontroleur:
    """
    Microcontrôleur monocœur et sa plateforme RTOS, avec un producteur et un consommateur.
    Une seule tâche s'exécute à la fois, sans préemption.
    """
    navigation: NavigationTask
    controle_moteurs: ControleMoteursTask

    def __init__(self, gnss: ModuleGNSS, imu: ModuleIMU, drone: Drone, flight_plan: list[Vector3],
                 navigation_config: TaskConfig, controle_config: TaskConfig):
        self.navigation = NavigationTask(gnss, imu, flight_plan, navigation_config)
        self.controle_moteurs = ControleMoteursTask(drone, imu, self.navigation, controle_config)
        # par ordre de priorité: le producteur avant le consommateur,
        # pour que le contrôle utilise la vitesse la plus récente
        self.tasks: list[Task] = [self.navigation, self.controle_moteurs]

    def next_event_t(self) -> float:
        return min(task.next_event_t() for task in self.tasks)

    def step(self):
        t = self.next_event_t()
        task = min(self.tasks, key=lambda task: task.next_event_t())
        task.step()
        self.dispatch(t)

    def dispatch(self, t: float):
        """
        Si le CPU est libre, le donner à la tâche prête la plus prioritaire
        """
        if any(task.status == "calculating" for task in self.tasks):
            return
        for task in self.tasks:
            if task.status == "ready":
                task.start(t)
                return
