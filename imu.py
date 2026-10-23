from drone import Drone
from util import Vector3

class ModuleIMU:
    def __init__(self, drone: Drone):
        self.drone = drone

    def run(self, dt: float):
        pass

    def get_accel(self) -> Vector3:
        return self.drone.accel_totale.copy() # TODO add noise
