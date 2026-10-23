from util import Vector3

class Drone:
    """
    Drone physique, moteurs et le driver de moteurs
    """
    position: Vector3

    MASSE = 1.5 # kg

    def __init__(self, vent: Vector3 | None = None):
        # vent statique: une accélération constante (m/s/s)
        self.vent = vent if vent is not None else Vector3(0.0, 0.0, 0.0)
        self.position = Vector3(0.0, 0.0, 0.0)
        self.vitesse = Vector3(0.0, 0.0, 0.0)
        self.accel = Vector3(0.0, 0.0, 0.0)
        self.force_cmd = Vector3(0.0, 0.0, 0.0) # consigne des moteurs (N)
        self.accel_totale = self.vent.copy() # moteurs + vent
        self.tau_moteurs = 0.5 # constante de temps des moteurs (s)

    def set_force(self, force: Vector3):
        self.force_cmd = force

    def run(self, dt: float):
        # les moteurs suivent la commande avec un retard du premier ordre, a = F/m
        alpha = dt / (self.tau_moteurs + dt)
        self.accel += (self.force_cmd / Drone.MASSE - self.accel) * alpha

        self.accel_totale = self.accel + self.vent
        self.position += self.vitesse * dt + (self.accel_totale/2.0) * (dt ** 2)
        self.vitesse += self.accel_totale * dt

    def get_position(self) -> Vector3:
        return self.position
