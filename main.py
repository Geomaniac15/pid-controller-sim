start_pos = [0,0]
velocity = [2,2]

pos = start_pos

acceleration = 3
dt = 0.01

for x in range(100):
    pos[0] += velocity[0]
    pos[1] += velocity[1]

    velocity *= acceleration

    print(f'Position: ({pos[0]},{pos[1]})')
    print(f'Velocity: ({velocity[0]},{velocity[1]})')
    print()