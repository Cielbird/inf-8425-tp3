import random

from drone import Drone
from util import Vector3

class ModuleIMU:
    BRUIT = 0.05 # écart-type du bruit

    def __init__(self, drone: Drone):
        self.drone = drone
        seed = 0
        self.rng = random.Random(seed)
        # vitesse intégrée par l'IMU depuis la dernière lecture (sortie "delta-v"):
        # l'IMU échantillonne bien plus vite que les tâches du MCU
        self.delta_v = Vector3(0.0, 0.0, 0.0)

    def bruit(self) -> Vector3:
        return Vector3(
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
            self.rng.gauss(0.0, ModuleIMU.BRUIT),
        )

    def run(self, dt: float):
        # appelé après drone.run: accel_totale est l'accélération appliquée pendant dt
        self.delta_v += (self.drone.accel_totale + self.bruit()) * dt

    def get_accel(self) -> Vector3:
        return self.drone.accel_totale + self.bruit()

    def get_delta_v(self) -> Vector3:
        """
        Variation de vitesse depuis la dernière lecture, puis remise à zéro
        """
        delta_v = self.delta_v
        self.delta_v = Vector3(0.0, 0.0, 0.0)
        return delta_v
