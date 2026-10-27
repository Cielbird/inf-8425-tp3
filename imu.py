import random

from drone import Drone
from util import Vector3

class ModuleIMU:
    BRUIT = 0.05 # écart-type du bruit

    def __init__(self, drone: Drone):
        self.drone = drone
        seed = 0
        self.rng = random.Random(seed)

    def run(self, dt: float):
        pass

    def get_accel(self) -> Vector3:
        bruit = Vector3(
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
        )
        return self.drone.accel_totale + bruit
