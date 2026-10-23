from dataclasses import dataclass

from drone import Drone
from gnss import ModuleGNSS;
from imu import ModuleIMU;
from gnss import ModuleGNSS
from mcu import Microcontroleur
from util import Vector3, plot_path, save_csv;

class System:
    def __init__(self):
        self.drone = Drone()
        self.gnss = ModuleGNSS(self.drone)
        self.imu = ModuleIMU(self.drone)
        self.mcu = Microcontroleur(self.gnss, self.imu, self.drone)
        self.t = 0.0

    def next_event_t(self) -> float:
        return self.mcu.next_event_t()

    def step(self):
        next_t = self.mcu.next_event_t()
        dt = next_t - self.t
        self.t = next_t

        # continuous time sim blocks
        self.gnss.run(dt)
        self.imu.run(dt)
        self.drone.run(dt)

        # discrete time sim block
        self.mcu.step()

def main():
    print("INF8425: Systèmes Cyber-Physiques, TP3, Simulation du logiciel GNSS.")

    system = System()

    # en secondes
    duration = 2500.0
    elapsed = 0.0
    t_log: list[float] = []
    pos_log: list[Vector3] = []

    while elapsed < duration:
        next_t = system.next_event_t()
        system.step()
        elapsed = next_t
        t_log.append(elapsed)
        pos_log.append(system.drone.position.copy())
        print(f"Simulation t={elapsed}")

    csv_path = "results.csv"
    save_csv(csv_path, t_log, pos_log)
    print(f"Simulation terminée, résultats dans: {csv_path}")

    plot_path(t_log, pos_log, system.mcu.navigation.flight_plan)

main()
