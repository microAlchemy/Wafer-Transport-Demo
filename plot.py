import matplotlib.pyplot as plt

from wafer_transport_sim.four_room_layout import ROOMS, PATH, START


fig, ax = plt.subplots(figsize=(10, 8))


# -------------------------------------------------
# DRAW ROOMS
# -------------------------------------------------

for room in ROOMS:

    # Same geometry used by generate_four_rooms.py
    ymin, ymax = sorted((
        room.y - room.direction * 0.30,
        room.y + room.direction * 0.65
    ))

    xmin = room.x - 0.5
    xmax = room.x + 0.5

    rect = plt.Rectangle(
        (xmin, ymin),
        xmax - xmin,
        ymax - ymin,
        edgecolor="blue",
        facecolor="none",
        linewidth=2
    )

    ax.add_patch(rect)

    # Room label
    ax.text(
        room.x,
        (ymin + ymax) / 2,
        f"Room {room.number}",
        ha="center",
        va="center",
        fontsize=11
    )

    # Entry door
    entry_x = room.door_x("entry")

    ax.plot(
        entry_x,
        room.y,
        "ro",
        markersize=8
    )

    # Exit door
    exit_x = room.door_x("exit")

    ax.plot(
        exit_x,
        room.y,
        "go",
        markersize=8
    )


# -------------------------------------------------
# DRAW ACTUAL TAPE PATH
# -------------------------------------------------

xs = [p[0] for p in PATH]
ys = [p[1] for p in PATH]

ax.plot(
    xs,
    ys,
    "k-",
    linewidth=2,
    label="Tape path"
)


# -------------------------------------------------
# START POSITION
# -------------------------------------------------

ax.plot(
    START[0],
    START[1],
    "mo",
    markersize=10,
    label="Start"
)


# -------------------------------------------------
# PLOT SETTINGS
# -------------------------------------------------

ax.set_aspect("equal", adjustable="box")

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")

ax.set_title("Four-Room Wafer Transport Layout")

ax.grid(True, alpha=0.3)

# Give some space around the track
ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.8, 1.8)

# Custom legend without repeating every door marker
from matplotlib.lines import Line2D

legend_items = [
    Line2D([0], [0], color="black", lw=2, label="Tape"),
    Line2D([0], [0], marker="o", color="w",
           markerfacecolor="red", markersize=8,
           label="Entry door"),
    Line2D([0], [0], marker="o", color="w",
           markerfacecolor="green", markersize=8,
           label="Exit door"),
    Line2D([0], [0], marker="o", color="w",
           markerfacecolor="magenta", markersize=8,
           label="Robot start"),
]

ax.legend(handles=legend_items)

plt.tight_layout()

plt.savefig(
    "path2.png",
    dpi=200
)

plt.show()