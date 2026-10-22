

class Drone:
    """
    Drone physique, moteurs et le driver de moteurs
    """
    def __init__(self):
        self.position = (0.0, 0.0, 0.0)
        self.accel = (0.0, 0.0, 0.0)

    def set_consigne(self, accel: tuple[float, float, float]):
        self.accel = accel

    def run(self, dt: float):
        pass

    def get_position(self) -> tuple[float, float, float]:
        return self.position
