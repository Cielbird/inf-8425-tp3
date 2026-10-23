from dataclasses import dataclass, replace
import csv
import math

@dataclass
class Vector3:
    """
    3D vector
    """
    x: float
    y: float
    z: float

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other: Vector3):
        return Vector3(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z
        )

    def __iadd__(self, other: Vector3):
        return self.__add__(other)

    def __sub__(self, other: Vector3):
        return Vector3(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z
        )

    def __mul__(self, other: Vector3 | float) -> Vector3:
        if type(other) is Vector3:
            return Vector3(
                self.x * other.x,
                self.y * other.y,
                self.z * other.z
            )
        elif type(other) is float:
            return Vector3(
                self.x * other,
                self.y * other,
                self.z * other
            )
        raise TypeError()

    def __truediv__(self, other: float) -> Vector3:
        return Vector3(
            self.x / other,
            self.y / other,
            self.z / other
        )

    def copy(self):
        return replace(self)

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def clamp(self, min_vec: Vector3, max_vec: Vector3):
        if self.x < min_vec.x:
            self.x = min_vec.x
        elif self.x > max_vec.x:
            self.x = max_vec.x

        if self.y < min_vec.y:
            self.y = min_vec.y
        elif self.y > max_vec.y:
            self.y = max_vec.y
            
        if self.z < min_vec.z:
            self.z = min_vec.z
        elif self.z > max_vec.z:
            self.z = max_vec.z

class PID3:
    """
    PID in 3D
    """
    p: float
    i: float
    d: float

    def __init__(self, p: float, i: float, d: float, dt: float, integrator_min: float, integrator_max: float) -> None:
        self.p = p
        self.i = i
        self.d = d
        self.dt = dt
        self.integrator_min = Vector3(integrator_min, integrator_min, integrator_min)
        self.integrator_max = Vector3(integrator_max, integrator_max, integrator_max)

        self._integrator = Vector3(0.0, 0.0, 0.0)
        self._old_err = Vector3(0.0, 0.0, 0.0)

    def __call__(self, value: Vector3, setpoint: Vector3) -> Vector3:
        err = setpoint - value
        p = err * self.p
        self._integrator += err * self.dt
        self._integrator.clamp(self.integrator_min, self.integrator_max)
        i = self._integrator.copy()  * self.i
        d = (err - self._old_err) / self.dt * self.d

        self._old_err = err.copy()

        return p + i + d


def save_csv(path: str, t_log: list[float], pos_log: list[Vector3]):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y", "z"])
        for t, p in zip(t_log, pos_log):
            writer.writerow([t, p.x, p.y, p.z])
