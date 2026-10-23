from util import Vector3

class Drone:
    """
    Drone physique, moteurs et le driver de moteurs
    """
    position: Vector3

    def __init__(self, vent: Vector3 | None = None):
        # vent statique: une accélération constante (m/s/s)
        self.vent = vent if vent is not None else Vector3(0.0, 0.0, 0.0)
        self.position = Vector3(0.0, 0.0, 0.0)
        self.vitesse = Vector3(0.0, 0.0, 0.0)
        self.accel = Vector3(0.0, 0.0, 0.0)
        self.accel_cmd = Vector3(0.0, 0.0, 0.0)
        self.accel_totale = self.vent.copy() # moteurs + vent
        self.tau_moteurs = 0.5 # constante de temps des moteurs (s)

    def set_accel(self, accel: Vector3):
        self.accel_cmd = accel

    def run(self, dt: float):
        # les moteurs suivent la commande avec un retard du premier ordre
        alpha = dt / (self.tau_moteurs + dt)
        self.accel += (self.accel_cmd - self.accel) * alpha

        self.accel_totale = self.accel + self.vent
        self.position += self.vitesse * dt + (self.accel_totale/2.0) * (dt ** 2)
        self.vitesse += self.accel_totale * dt

    def get_position(self) -> Vector3:
        return self.position
