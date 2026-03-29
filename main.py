import matplotlib.pyplot as plt 
import numpy as np

pos = [0.0, 0.0]
velocity = [2.0, 2.0]
acceleration = [0.0, 0.0]

x_pos = []
y_pos = []

dt = 0.01

for step in range(100):
    velocity[0] += acceleration[0] * dt
    velocity[1] += acceleration[1] * dt

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    x_pos.append(pos[0])
    y_pos.append(pos[1])

    # print(f'Position: ({pos[0]},{pos[1]})')
    # print(f'Velocity: ({velocity[0]},{velocity[1]})')
    # print()

xpoints = np.array(x_pos)
ypoints = np.array(y_pos)

plt.scatter(xpoints, ypoints, s=2)

plt.xlabel('X Velocity')
plt.ylabel('Y Velocity')

plt.show()