from drone import Drone
from gnss import ModuleGNSS
from imu import ModuleIMU
from mcu import Microcontroleur
from graph import plot_mcu_usage, plot_path
from util import Vector3, save_csv

class System:
    def __init__(self):
        self.drone = Drone(vent=Vector3(0.1, 0.05, 0.001))
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
    duration = 80.0
    elapsed = 0.0
    t_log: list[float] = []
    pos_log: list[Vector3] = []
    sat_log: list[int] = []

    # pannes GNSS: (début, fin, nombre de satellites), en s
    pannes = [
        (15.0, 20.0, 3), # fix 2D pendant la montée: altitude figée -> écart vertical
        (45.0, 50.0, 1), # pas de fix en palier: position figée -> écart horizontal
        (35.0, 40.0, 2), # pas de fix en palier: position figée -> écart horizontal
    ]
    navigation = system.mcu.navigation
    etape = navigation.next_checkpoint_idx

    while elapsed < duration:
        satelites = 4
        for debut, fin, n in pannes:
            if debut <= elapsed < fin:
                satelites = n
        system.gnss.set_satelites(satelites)

        if navigation.next_checkpoint_idx != etape:
            etape = navigation.next_checkpoint_idx
            print(f"Étape {etape} à t={elapsed:.2f}")

        next_t = system.next_event_t()
        system.step()
        elapsed = next_t
        t_log.append(elapsed)
        pos_log.append(system.drone.position.copy())
        sat_log.append(system.gnss.satelites)
        print(f"Simulation t={elapsed}")

    csv_path = "results.csv"
    save_csv(csv_path, t_log, pos_log)
    print(f"Simulation terminée, résultats dans: {csv_path}")

    mcu = system.mcu
    plot_mcu_usage(
        {
            "Navigation": mcu.navigation.exec_log,
            "Contrôle moteurs": mcu.controle_moteurs.exec_log,
        },
        mcu.navigation.period,
    )
    # plot_path appelle plt.show(), qui affiche aussi la figure du MCU
    plot_path(t_log, pos_log, mcu.navigation.flight_plan, sat_log)

if __name__ == "__main__":
    main()
