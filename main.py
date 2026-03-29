import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import cos, sin, sqrt
import random

pos = [0.0, 0.0]
velocity = [0.0, 0.0]

max_speed = 2.0

centre_x = 1.5
centre_y = 3.0
R = 0.5

Kp = 2.0 # proportional - accelerator
Kd = 1.5 # derivative - brakes

noisy_x_pos = []
noisy_y_pos = []
measured_x_pos = []
measured_y_pos = []

alpha = 0.05
dt = 0.01

# initialise filtered state
filtered_x = 0.0
filtered_y = 0.0

def update(frame):
    t = frame * dt

    x_target = centre_x + R * cos(t)
    y_target = centre_y + R * sin(t)

    # add measurement noise
    noise_level = 0.2
    measured_x = pos[0] + random.uniform(-noise_level, noise_level)
    measured_y = pos[1] + random.uniform(-noise_level, noise_level)

    global filtered_x, filtered_y

    filtered_x = alpha * measured_x + (1 - alpha) * filtered_x
    filtered_y = alpha * measured_y + (1 - alpha) * filtered_y

    ax_control = Kp * (x_target - filtered_x) - Kd * velocity[0]
    ay_control = Kp * (y_target - filtered_y) - Kd * velocity[1]

    acceleration = [ax_control, ay_control]

    velocity[0] += acceleration[0] * dt
    velocity[1] += acceleration[1] * dt

    speed = sqrt((velocity[0]**2) + (velocity[1]**2))
    # print(speed)
    if speed > max_speed:
        scale = max_speed / speed
        velocity[0] *= scale
        velocity[1] *= scale

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    noisy_x_pos.append(pos[0])
    noisy_y_pos.append(pos[1])
    measured_x_pos.append(measured_x)
    measured_y_pos.append(measured_y)

    line.set_data(noisy_x_pos, noisy_y_pos)
    measured_line.set_data(measured_x_pos, measured_y_pos)
    target_point.set_data([x_target], [y_target])

    return line, measured_line, target_point

fig, ax = plt.subplots()

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)

line, = ax.plot([], [])
measured_line, = ax.plot([], [], 'g--', alpha=0.6)
target_point, = ax.plot([], [], 'ro', markersize=5)

ax.legend(['Actual Path', 'Measured Path', 'Target'])

ani = FuncAnimation(fig, update, frames=500, interval=20, blit=True)
# ani.save('output.mp4', fps=30)

plt.xlabel('X Pos')
plt.ylabel('Y Pos')

plt.show()