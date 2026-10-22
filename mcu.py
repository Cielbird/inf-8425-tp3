from drone import Drone
from gnss import ModuleGNSS
from imu import ModuleIMU


class NavigationTask:
    """
    5Hz:
    Read from GNSS module with UART, calculate necessary velocity, then write value to shared memory
    """

    def __init__(self, gnss: ModuleGNSS):
        self.gnss = gnss
        self.period = 0.05 # 5Hz frequency
        # read and writing times are negligible
        self.processing_dt = 0.05 # calculating time in seconds
        self.status = "waiting" # in {"waiting", "calculating"}
        self.elapsed = 0.0

        # task variables
        self.position = (0.0, 0.0, 0.0) # latitude, longitude, altitude
        self.velocity = (0.0, 0.0, 0.0) # "shared" memory
        pass


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

            case "calculating":
                # read from GNSS
                self.position = self.gnss.get_position()
                # calculation time
                self.elapsed = next_t
                # write to shared mem
                self.velocity = self.calculate_vel()

            case _:
                raise TypeError("Invalid status")

    def calculate_vel(self) -> tuple[float, float, float]:
        return (1.0, 1.0, 1.0)
            


class ControleMoteursTask:
    def __init__(self, drone: Drone, imu: ModuleIMU, nav_task: NavigationTask):
        self.drone = drone
        self.imu = imu
        self.nav_task = nav_task
        self.period = 0.05 # 5Hz frequency
        # read and writing times are negligible
        self.processing_dt = 0.05 # calculating time in seconds
        self.status = "waiting" # in {"waiting", "calculating"}
        self.elapsed = 0.0

        # task variables
        self.acceleration = (0.0, 0.0, 0.0) # latitude, longitude, altitude TODO
        pass


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

            case "calculating":
                # read from GNSS
                vel = self.nav_task.velocity

                accel = self.calculate_consigne(vel)

                # calculation time
                self.elapsed = next_t
                # write to shared mem
                self.drone.set_consigne(accel) # TODO
                
            case _: raise TypeError("invalid control moteurs task state")

    def calculate_consigne(self, vel: tuple[float, float, float]) -> tuple[float, float, float]:
        _accel = self.imu.get_accel()
        # TODO PID
        return (0.0, 0.0, 0.0)

class Microcontroleur:
    """
    Microcontrôleur et sa plateforme RTOS, avec un producteur et un consommateur
    """
    navigation: NavigationTask
    controle_moteurs: ControleMoteursTask
    
    def __init__(self, gnss: ModuleGNSS, drone: Drone):
        self.navigation = NavigationTask(gnss)
        self.controle_moteurs = ControleMoteursTask(drone, self.navigation)

    def next_event_t(self) -> float:
        min(self.navigation.next_event_t(), self.controle_moteurs.next_event_t())

    def step(self):
        nav_next_t = self.navigation.next_event_t()
        controle_next_t = self.controle_moteurs.next_event_t()
        if nav_next_t > controle_next_t:
            self.controle_moteurs.step()
        else:
            self.navigation.step()
