from dataclasses import dataclass

from drone import Drone
from gnss import ModuleGNSS;
from imu import ModuleIMU;
from gnss import ModuleGNSS
from mcu import Microcontroleur;

@dataclass
class SimStatus:
    pass

class Alimentation:
    def __init__(self):
        pass

class System:
    def __init__(self):
        self.gnss = ModuleGNSS()
        self.imu = ModuleIMU()
        self.drone = Drone()
        self.mcu = Microcontroleur(self.gnss, self.drone)

    def next_event_t(self) -> float:
        return self.mcu.next_event_t()

    def step(self):
        print("Simulation en cours... TODO")

        dt = self.mcu.next_event_t()

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
    duration = 1.0
    elapsed = 0.0
    
    log = []

    while (elapsed < duration):
        next_t = system.next_event_t()
        stat = system.step()
        elapsed = next_t
        log.append(stat)

    # save `log` to csv
    print("Simulation terminée, résultats dans: TODO.csv")



if __name__ == "__main__":
    main()
