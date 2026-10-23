from util import Vector3

class Drone:
    """
    Drone physique, moteurs et le driver de moteurs
    """
    position: Vector3

    def __init__(self):
        self.position = Vector3(0.0, 0.0, 0.0)
        self.vitesse = Vector3(0.0, 0.0, 0.0)
        self.accel = Vector3(0.0, 0.0, 0.0)

    def set_accel(self, accel: Vector3):
        self.accel = accel

    def run(self, dt: float):
        self.position += self.vitesse * dt + (self.accel/2.0) * (dt ** 2)
        self.vitesse += self.accel * dt

    def get_position(self) -> Vector3:
        return self.position
