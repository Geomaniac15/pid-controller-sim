import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation

pos = [0.0, 0.0]
velocity = [0.0, 0.0]

x_target = 4.0
y_target = 4.0

Kp = 1.5 # proportional - accelerator
Kd = 2.0 # derivative - brakes

x_pos = []
y_pos = []

dt = 0.01

def update(frame):
    ax_control = Kp * (x_target - pos[0]) - Kd * velocity[0]
    ay_control = Kp * (y_target - pos[1]) - Kd * velocity[1]

    acceleration = [ax_control, ay_control]

    velocity[0] += acceleration[0] * dt
    velocity[1] += acceleration[1] * dt

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    x_pos.append(pos[0])
    y_pos.append(pos[1])

    line.set_data(x_pos, y_pos)

    # print(f'Position: ({pos[0]},{pos[1]})')
    # print(f'Velocity: ({velocity[0]},{velocity[1]})')
    # print()

    return line,

fig, ax = plt.subplots()

ax.plot(x_target, y_target, 'ro', markersize=5)

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)

line, = ax.plot([], [])

ani = FuncAnimation(fig, update, frames=200, interval=50)

plt.xlabel('X Pos')
plt.ylabel('Y Pos')

plt.show()