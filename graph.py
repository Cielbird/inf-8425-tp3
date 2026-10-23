import math
import matplotlib.pyplot as plt

from util import Vector3

ECART_MAX = 3.0 # écart maximal toléré par rapport au plan de vol (m)

def distance_segment(p: Vector3, a: Vector3, b: Vector3) -> float:
    """
    Distance entre le point p et le segment [a, b]
    """
    ab = b - a
    ap = p - a
    longueur2 = ab.x ** 2 + ab.y ** 2 + ab.z ** 2
    if longueur2 == 0.0:
        return ap.magnitude()
    # projection de p sur la droite (a, b), bornée au segment
    s = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / longueur2
    s = min(max(s, 0.0), 1.0)
    return (ap - ab * s).magnitude()

def ecart_plan(p: Vector3, flight_plan: list[Vector3]) -> float:
    """
    Distance entre le point p et le segment le plus proche du plan de vol
    """
    return min(distance_segment(p, a, b) for a, b in zip(flight_plan, flight_plan[1:]))

def intervalles(t_log: list[float], masque: list[bool]) -> list[tuple[float, float]]:
    """
    Intervalles de temps (début, fin) pendant lesquels le masque est vrai
    """
    result: list[tuple[float, float]] = []
    debut = None
    for t, m in zip(t_log, masque):
        if m and debut is None:
            debut = t
        elif not m and debut is not None:
            result.append((debut, t))
            debut = None
    if debut is not None:
        result.append((debut, t_log[-1]))
    return result

def plot_path(t_log: list[float], pos_log: list[Vector3], flight_plan: list[Vector3], sat_log: list[int]):
    xs = [p.x for p in pos_log]
    ys = [p.y for p in pos_log]
    zs = [p.z for p in pos_log]

    def portions(masque: list[bool]) -> tuple[list[float], list[float], list[float]]:
        # garde les points où le masque est vrai, NaN ailleurs pour couper la ligne;
        # on garde aussi le point qui suit pour que la portion soit continue
        garde = [m or (i > 0 and masque[i - 1]) for i, m in enumerate(masque)]
        return (
            [x if g else math.nan for x, g in zip(xs, garde)],
            [y if g else math.nan for y, g in zip(ys, garde)],
            [z if g else math.nan for z, g in zip(zs, garde)],
        )

    fix_2d_xs, fix_2d_ys, fix_2d_zs = portions([n == 3 for n in sat_log])
    lost_xs, lost_ys, lost_zs = portions([n < 3 for n in sat_log])

    plan_xs = [p.x for p in flight_plan]
    plan_ys = [p.y for p in flight_plan]
    plan_zs = [p.z for p in flight_plan]

    # checkpoints at the same position share one label, e.g. "0, 5"
    checkpoint_labels: dict[tuple[float, float, float], list[str]] = {}
    for idx, p in enumerate(flight_plan):
        checkpoint_labels.setdefault((p.x, p.y, p.z), []).append(str(idx))

    # 3D trajectory
    fig3d = plt.figure(figsize=(7, 6))
    ax3d = fig3d.add_subplot(projection="3d")
    ax3d.plot(plan_xs, plan_ys, plan_zs, "k:", label="Plan de vol")
    ax3d.scatter(plan_xs, plan_ys, plan_zs, c="k", marker="X", s=60, label="Points de passage") # type: ignore
    for (x, y, z), idxs in checkpoint_labels.items():
        ax3d.text(x, y, z, "  " + ", ".join(idxs)) # type: ignore
    ax3d.plot(xs, ys, zs, label="Trajectoire")
    ax3d.plot(fix_2d_xs, fix_2d_ys, fix_2d_zs, c="orange", label="Fix 2D (3 satelites)")
    ax3d.plot(lost_xs, lost_ys, lost_zs, c="r", label="Aucun fix (<3 satelites)")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    # même échelle sur les 3 axes, sinon un axe presque plat est agrandi
    all_x, all_y, all_z = xs + plan_xs, ys + plan_ys, zs + plan_zs
    half_range = max(max(a) - min(a) for a in (all_x, all_y, all_z)) / 2
    for set_lim, a in ((ax3d.set_xlim, all_x), (ax3d.set_ylim, all_y), (ax3d.set_zlim, all_z)): # type: ignore
        center = (max(a) + min(a)) / 2
        set_lim(center - half_range, center + half_range)
    ax3d.set_box_aspect((1, 1, 1)) # type: ignore
    ax3d.set_title("Trajectoire du drone")
    ax3d.legend()
    fig3d.tight_layout()

    # Écart par rapport au plan de vol
    ecarts = [ecart_plan(p, flight_plan) for p in pos_log]
    fig_ecart, ax_ecart = plt.subplots(figsize=(12, 4))

    # a) périodes de perte du fix GNSS, en fond
    pertes = [
        ("3 satelites (fix 2D)", "yellow", [n == 3 for n in sat_log]),
        ("<3 satelites (aucun fix)", "orange", [n < 3 for n in sat_log]),
    ]
    for label, couleur, masque in pertes:
        for i, (debut, fin) in enumerate(intervalles(t_log, masque)):
            ax_ecart.axvspan(debut, fin, color=couleur, alpha=0.2, label=label if i == 0 else None)

    ax_ecart.plot(t_log, ecarts, label="Écart")

    # b) zones où l'écart dépasse la limite
    ax_ecart.axhline(ECART_MAX, color="r", linestyle="--", label=f"Limite {ECART_MAX:g} m")
    ax_ecart.fill_between(
        t_log, ECART_MAX, ecarts,
        where=[e > ECART_MAX for e in ecarts], color="r", alpha=0.4, label=f"Écart > {ECART_MAX:g} m",
    )

    ax_ecart.set_xlabel("t (s)")
    ax_ecart.set_ylabel("écart (m)")
    ax_ecart.set_title("Écart par rapport au plan de vol")
    ax_ecart.set_xlim(t_log[0], t_log[-1])
    ax_ecart.set_ylim(bottom=0)
    ax_ecart.grid(True)
    ax_ecart.legend(loc="upper right")
    fig_ecart.tight_layout()

    plt.show()

def plot_mcu_usage(exec_logs: dict[str, list[tuple[float, float]]], period: float, n_cycles: int = 10):
    """
    Chronogramme d'utilisation du MCU: une ligne et une couleur par tâche.
    N'affiche pas la figure; plt.show() s'en charge.
    """
    t_max = n_cycles * period
    fig, ax = plt.subplots(figsize=(12, 3))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for row, (name, log) in enumerate(exec_logs.items()):
        bars = [(start, dur) for start, dur in log if start < t_max]
        ax.broken_barh(bars, (row - 0.4, 0.8), facecolors=colors[row % len(colors)], label=name)

    ax.set_yticks(range(len(exec_logs)), list(exec_logs.keys()))
    ax.set_xlim(0, t_max)
    ax.set_xlabel("t (s)")
    ax.set_title(f"Utilisation du MCU ({n_cycles} cycles)")
    ax.grid(True, axis="x")
    ax.legend(loc="upper right")
    fig.tight_layout()
