import matplotlib.pyplot as plt 

pos = [0.0, 0.0]
velocity = [2.0, 2.0]
acceleration = [0.0, 0.0]

dt = 0.01

for step in range(100):
    velocity[0] += acceleration[0] * dt
    velocity[1] += acceleration[1] * dt

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    print(f'Position: ({pos[0]},{pos[1]})')
    print(f'Velocity: ({velocity[0]},{velocity[1]})')
    print()