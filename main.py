import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import cos, sin, sqrt

pos = [0.0, 0.0]
velocity = [0.0, 0.0]

max_speed = 2.0

centre_x = 4.0
centre_y = 4.0
R = 0.5

Kp = 2.5 # proportional - accelerator
Kd = 2.0 # derivative - brakes

x_pos = []
y_pos = []

dt = 0.01

def update(frame):
    t = frame * dt

    x_target = centre_x + R * cos(t)
    y_target = centre_y + R * sin(t)

    ax_control = Kp * (x_target - pos[0]) - Kd * velocity[0]
    ay_control = Kp * (y_target - pos[1]) - Kd * velocity[1]

    acceleration = [ax_control, ay_control]

    velocity[0] += acceleration[0] * dt
    velocity[1] += acceleration[1] * dt

    speed = sqrt((velocity[0]**2) + (velocity[1]**2))
    print(speed)
    if speed > max_speed:
        scale = max_speed / speed
        velocity[0] *= scale
        velocity[1] *= scale

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    x_pos.append(pos[0])
    y_pos.append(pos[1])

    line.set_data(x_pos, y_pos)
    target_point.set_data([x_target], [y_target])

    return line, target_point

fig, ax = plt.subplots()

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)

line, = ax.plot([], [])
target_point, = ax.plot([], [], 'ro', markersize=5)

ani = FuncAnimation(fig, update, frames=500, interval=20, blit=True)

plt.xlabel('X Pos')
plt.ylabel('Y Pos')

plt.show()