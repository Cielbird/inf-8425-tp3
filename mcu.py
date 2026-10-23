from drone import Drone
from gnss import ModuleGNSS
from imu import ModuleIMU
from util import PID3, Vector3


class NavigationTask:
    """
    5Hz:
    Read from GNSS module with UART, calculate necessary velocity, then write value to shared memory
    """

    def __init__(self, gnss: ModuleGNSS):
        self.gnss = gnss
        self.period = 0.05 # 5Hz frequency
        # read and writing times are negligible
        self.processing_dt = 0.005 # calculating time in seconds
        self.status = "waiting" # in {"waiting", "calculating"}
        self.elapsed = 0.0

        # task variables
        self.position = Vector3(0.0, 0.0, 0.0) # latitude, longitude, altitude
        self.velocity = Vector3(0.0, 0.0, 0.0) # "shared" memory

        self.next_checkpoint_idx = 1
        self.flight_plan = [
            Vector3(0.0, 0.0, 0.0),
            Vector3(30.0, 0.0, 0.0),
            Vector3(0.0, 30.0, 0.0),
            Vector3(-30.0, 0.0, 0.0),
            Vector3(0.0, -30.0, 0.0),
            Vector3(0.0, 0.0, 0.0),
        ]


    def next_event_t(self) -> float:
        """
        Get the time (seconds) until the next event
        """
        match self.status:
            case "waiting":
                return self.elapsed + self.period - self.processing_dt
            case "calculating":
                return self.elapsed + self.processing_dt
            case _:
                raise TypeError("Invalid status")
                
    def step(self):
        """
        Run sim until next event
        """
        next_t = self.next_event_t()
        match self.status:
            case "waiting":
                # wait until next calc
                self.elapsed = next_t

                self.status = "calculating"

            case "calculating":
                # read from GNSS
                self.position = self.gnss.get_position()
                # calculation time
                self.elapsed = next_t
                # write to shared mem
                self.velocity = self.calculate_vel()
                self.status = "waiting"

            case _:
                raise TypeError("Invalid status")

    def calculate_vel(self) -> Vector3:
        err = self.flight_plan[self.next_checkpoint_idx] - self.gnss.get_position()
        if err.magnitude() < 0.1:
            if self.next_checkpoint_idx < len(self.flight_plan) - 1:
                self.next_checkpoint_idx += 1
        p = 0.0005
        return err / self.period * p


class ControleMoteursTask:
    def __init__(self, drone: Drone, imu: ModuleIMU, nav_task: NavigationTask):
        self.drone = drone
        self.imu = imu
        self.nav_task = nav_task
        self.period = 0.05 # 5Hz frequency
        # read and writing times are negligible
        self.processing_dt = 0.005 # calculating time in seconds
        self.status = "waiting" # in {"waiting", "calculating"}
        self.elapsed = 0.0

        # task variables
        self.vitesse_imu = Vector3(0.0, 0.0, 0.0) # intégrateur après le capteur IMU, avant PID
        self.pid = PID3(0.05, 0.0, 0.0, self.period, -10, 10)


    def next_event_t(self) -> float:
        """
        Get the time (seconds) until the next event
        """
        match self.status:
            case "waiting":
                return self.elapsed + self.period - self.processing_dt
            case "calculating":
                return self.elapsed + self.processing_dt
            case _: raise TypeError("invalid control moteurs task state")
                
    def step(self):
        """
        Run sim until next event
        """
        next_t = self.next_event_t()
        match self.status:
            case "waiting":
                # wait until next calc
                self.elapsed = next_t
                self.status = "calculating"

            case "calculating":
                # read from GNSS
                vel = self.nav_task.velocity

                accel_imu = self.imu.get_accel()
                self.vitesse_imu += accel_imu * self.period
                accel = self.pid(self.vitesse_imu, vel)

                # calculation time
                self.elapsed = next_t
                # write to shared mem
                self.drone.set_accel(accel)
                self.status = "waiting"
                
            case _: raise TypeError("invalid control moteurs task state")

class Microcontroleur:
    """
    Microcontrôleur et sa plateforme RTOS, avec un producteur et un consommateur
    """
    navigation: NavigationTask
    controle_moteurs: ControleMoteursTask
    
    def __init__(self, gnss: ModuleGNSS, imu: ModuleIMU, drone: Drone):
        self.navigation = NavigationTask(gnss)
        self.controle_moteurs = ControleMoteursTask(drone, imu, self.navigation)

    def next_event_t(self) -> float:
        return min(self.navigation.next_event_t(), self.controle_moteurs.next_event_t())

    def step(self):
        nav_next_t = self.navigation.next_event_t()
        controle_next_t = self.controle_moteurs.next_event_t()
        if nav_next_t > controle_next_t:
            self.controle_moteurs.step()
        else:
            self.navigation.step()
