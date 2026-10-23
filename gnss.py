from util import Vector3

from drone import Drone

class ModuleGNSS:
    """
    Module GNSS. Le type de fix dépend du nombre de satellites visibles:
    - 4+ satellites: fix 3D, position complète
    - 3 satellites: fix 2D, x et y à jour, l'altitude reste figée à la dernière valeur du fix 3D
    - moins de 3: pas de fix, la dernière position est renvoyée
    """

    def __init__(self, drone: Drone):
        self.drone = drone
        self.satelites = 4
        self.derniere_position = drone.get_position().copy()

    def set_satelites(self, n: int):
        self.satelites = n

    def run(self, dt: float):
        pass

    def get_position(self) -> Vector3:
        position = self.drone.get_position().copy() # TODO add errors depending on satelite
        if self.satelites >= 4:
            self.derniere_position = position
        elif self.satelites == 3:
            # fix 2D: le récepteur suppose que l'altitude n'a pas changé
            self.derniere_position = Vector3(position.x, position.y, self.derniere_position.z)
        return self.derniere_position.copy()
