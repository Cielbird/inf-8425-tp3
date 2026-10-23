from drone import Drone
from gnss import ModuleGNSS
from imu import ModuleIMU
from util import PID3, Vector3


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
    20Hz:
    Read from GNSS module with UART, calculate necessary velocity, then write value to shared memory
    """

    def __init__(self, gnss: ModuleGNSS):
        super().__init__(period=0.05, processing_dt=0.005) # 20Hz
        self.gnss = gnss

        # task variables
        self.position = Vector3(0.0, 0.0, 0.0) # x, y, z (m)
        self.velocity = Vector3(0.0, 0.0, 0.0) # "shared" memory

        self.next_checkpoint_idx = 1
        self.flight_plan = [
            Vector3(0.0, 0.0, 0.0),
            Vector3(0.0, 0.0, 10.0), # décollage
            Vector3(40.0, 0.0, 10.0),
            Vector3(40.0, 0.0, 25.0), # montée
            Vector3(40.0, 40.0, 25.0),
            Vector3(0.0, 40.0, 25.0),
            Vector3(0.0, 40.0, 0.0), # atterrissage
        ]
        self.pid = PID3(0.4, 0.001, 0.0, self.period, -10, 10)

    def compute(self):
        # read from GNSS
        self.position = self.gnss.get_position()
        # write to shared mem
        self.velocity = self.calculate_vel()

    def calculate_vel(self) -> Vector3:
        target = self.flight_plan[self.next_checkpoint_idx]
        if (target - self.position).magnitude() < 0.1:
            if self.next_checkpoint_idx < len(self.flight_plan) - 1:
                self.next_checkpoint_idx += 1
        return self.pid(self.position, target)


class ControleMoteursTask(Task):
    """
    20Hz:
    Read the navigation velocity from shared memory and the IMU, then command the motors
    """

    def __init__(self, drone: Drone, imu: ModuleIMU, nav_task: NavigationTask):
        super().__init__(period=0.05, processing_dt=0.005) # 20Hz
        self.drone = drone
        self.imu = imu
        self.nav_task = nav_task

        # task variables
        self.vitesse_imu = Vector3(0.0, 0.0, 0.0) # intégrateur après le capteur IMU, avant PID
        # intégrateur limité à ±2.5 (0.5 m/s² avec i=0.2): assez pour compenser le vent,
        # sans emballement (windup) pendant les grandes accélérations
        self.pid = PID3(1.0, 0.2, 0.0, self.period, -2.5, 2.5)

    def compute(self):
        # read from shared mem
        vel = self.nav_task.velocity

        accel_imu = self.imu.get_accel()
        self.vitesse_imu += accel_imu * self.period
        accel = self.pid(self.vitesse_imu, vel)

        # write to motors
        self.drone.set_accel(accel)


class Microcontroleur:
    """
    Microcontrôleur monocœur et sa plateforme RTOS, avec un producteur et un consommateur.
    Une seule tâche s'exécute à la fois, sans préemption.
    """
    navigation: NavigationTask
    controle_moteurs: ControleMoteursTask

    def __init__(self, gnss: ModuleGNSS, imu: ModuleIMU, drone: Drone):
        self.navigation = NavigationTask(gnss)
        self.controle_moteurs = ControleMoteursTask(drone, imu, self.navigation)
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
