from util import Vector3

from drone import Drone

class ModuleGNSS:

    def __init__(self, drone: Drone):
        self.drone = drone
        self.satelites = 4

    def run(self, dt: float):
        pass

    def get_position(self) -> Vector3:
        return self.drone.get_position() # TODO add errors depending on satelite
