import math
import matplotlib.pyplot as plt

from util import Vector3

def plot_path(t_log: list[float], pos_log: list[Vector3], flight_plan: list[Vector3], fix_3d_log: list[bool]):
    xs = [p.x for p in pos_log]
    ys = [p.y for p in pos_log]
    zs = [p.z for p in pos_log]

    # portions sans fix 3D, NaN ailleurs pour couper la ligne;
    # on garde aussi le point qui suit pour que le segment rouge soit continu
    lost = [not fix or (i > 0 and not fix_3d_log[i - 1]) for i, fix in enumerate(fix_3d_log)]
    lost_xs = [x if l else math.nan for x, l in zip(xs, lost)]
    lost_ys = [y if l else math.nan for y, l in zip(ys, lost)]
    lost_zs = [z if l else math.nan for z, l in zip(zs, lost)]

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
    ax3d.plot(lost_xs, lost_ys, lost_zs, c="r", label="Sans fix 3D")
    ax3d.scatter(xs[0], ys[0], zs[0], c="g", label="Départ") # type: ignore
    ax3d.scatter(xs[-1], ys[-1], zs[-1], c="r", label="Arrivée") # type: ignore
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

    # Side view (x-z)
    fig_side, ax2d = plt.subplots(figsize=(7, 5))
    ax2d.plot(plan_xs, plan_zs, "k:")
    ax2d.scatter(plan_xs, plan_zs, c="k", marker="X", s=60)
    for (x, _, z), idxs in checkpoint_labels.items():
        ax2d.annotate(", ".join(idxs), (x, z), textcoords="offset points", xytext=(6, 6))
    ax2d.plot(xs, zs)
    ax2d.plot(lost_xs, lost_zs, c="r")
    ax2d.scatter(xs[0], zs[0], c="g")
    ax2d.scatter(xs[-1], zs[-1], c="r")
    ax2d.set_xlabel("x (m)")
    ax2d.set_ylabel("z (m)")
    ax2d.set_title("Vue de côté")
    ax2d.axis("equal")
    ax2d.grid(True)
    fig_side.tight_layout()

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
