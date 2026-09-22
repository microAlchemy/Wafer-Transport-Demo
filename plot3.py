import matplotlib.pyplot as plt

from wafer_transport_sim.four_room_layout import PATH, ROOMS, START


fig, ax = plt.subplots(figsize=(10, 8))

# Draw rooms
for room in ROOMS:
    ymin, ymax = sorted((
        room.y - room.direction * 0.30,
        room.y + room.direction * 0.65
    ))

    rect = plt.Rectangle(
        (room.x - 0.5, ymin),
        1.0,
        ymax - ymin,
        fill=False,
        linewidth=2
    )

    ax.add_patch(rect)

    ax.text(
        room.x,
        (ymin + ymax) / 2,
        f"Room {room.number}",
        ha="center",
        va="center"
    )

# Draw actual Gazebo path
xs = [p[0] for p in PATH]
ys = [p[1] for p in PATH]

ax.plot(xs, ys, "k-", linewidth=2)

# Start
ax.plot(START[0], START[1], "o")

ax.set_aspect("equal")
ax.grid(True)

ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.8, 1.8)

plt.tight_layout()
plt.savefig("path3.png", dpi=200)
plt.show()