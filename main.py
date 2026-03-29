import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import numpy as np

pos = [0.0, 0.0]
velocity = [2.0, 2.0]
acceleration = [0.0, 0.0]

x_pos = []
y_pos = []

dt = 0.01

def update(frame):
    global line 
    
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

fig, ax = plt.subplots()

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)

line, = ax.plot([], [])

ani = FuncAnimation(fig, update, frames=200, interval=50)

plt.xlabel('X Pos')
plt.ylabel('Y Pos')

plt.show()