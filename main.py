import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import cos, sin, sqrt, inf
from statistics import mean
import random

pos = [0.0, 0.0]
velocity = [0.0, 0.0]

max_speed = 2.0

centre_x = 1.5
centre_y = 3.0
R = 0.5

Kp = 2.0 # proportional - accelerator
Kd = 2.5 # derivative - brakes
Ki = 0.0

noisy_x_pos = []
noisy_y_pos = []
measured_x_pos = []
measured_y_pos = []

alpha = 0.1
dt = 0.01
limit = 5

# initialise filtered state
filtered_x = 0.0
filtered_y = 0.0

# error of sums
error_sum_x = 0.0
error_sum_y = 0.0

def simulate(Kp, Kd, Ki=0.0, alpha=0.1):
    # reset state for each simulation
    pos = [0.0, 0.0]
    velocity = [0.0, 0.0]
    filtered_x = 0.0
    filtered_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0

    total_cost = 0.0

    for step in range(500):
        t = step * dt

        x_target = centre_x + R * cos(t)
        y_target = centre_y + R * sin(t)

        # noise
        noise_level = 0.2
        measured_x = pos[0] + random.uniform(-noise_level, noise_level)
        measured_y = pos[1] + random.uniform(-noise_level, noise_level)

        # filtering
        filtered_x = alpha * measured_x + (1 - alpha) * filtered_x
        filtered_y = alpha * measured_y + (1 - alpha) * filtered_y

        # error
        error_x = x_target - filtered_x
        error_y = y_target - filtered_y

        # PID
        ax_control = Kp * error_x + Ki * error_sum_x - Kd * velocity[0]
        ay_control = Kp * error_y + Ki * error_sum_y - Kd * velocity[1]

        velocity[0] += ax_control * dt
        velocity[1] += ay_control * dt

        speed = sqrt(velocity[0]**2 + velocity[1]**2)
        if speed > max_speed:
            scale = max_speed / speed
            velocity[0] *= scale
            velocity[1] *= scale

        pos[0] += velocity[0] * dt
        pos[1] += velocity[1] * dt

        # integral
        error_sum_x += error_x * dt
        error_sum_y += error_y * dt

        error_sum_x = max(min(error_sum_x, limit), -limit)
        error_sum_y = max(min(error_sum_y, limit), -limit)

        # accumulate cost
        total_cost += (x_target - pos[0])**2 + (y_target - pos[1])**2

    return total_cost

def update(frame):
    t = frame * dt

    x_target = centre_x + R * cos(t)
    y_target = centre_y + R * sin(t)

    # add measurement noise
    noise_level = 0.2
    measured_x = pos[0] + random.uniform(-noise_level, noise_level)
    measured_y = pos[1] + random.uniform(-noise_level, noise_level)

    global filtered_x, filtered_y, error_sum_x, error_sum_y

    filtered_x = alpha * measured_x + (1 - alpha) * filtered_x
    filtered_y = alpha * measured_y + (1 - alpha) * filtered_y

    # compute error using filtered measurements
    error_x = x_target - filtered_x
    error_y = y_target - filtered_y

    # PID control
    ax_control = Kp * error_x + Ki * error_sum_x - Kd * velocity[0]
    ay_control = Kp * error_y + Ki * error_sum_y - Kd * velocity[1]

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

    error_sum_x += error_x * dt
    error_sum_y += error_y * dt

    error_sum_x = max(min(error_sum_x, limit), -limit)
    error_sum_y = max(min(error_sum_y, limit), -limit)

    noisy_x_pos.append(pos[0])
    noisy_y_pos.append(pos[1])
    measured_x_pos.append(measured_x)
    measured_y_pos.append(measured_y)

    line.set_data(noisy_x_pos, noisy_y_pos)
    measured_line.set_data(measured_x_pos, measured_y_pos)
    target_point.set_data([x_target], [y_target])

    return line, measured_line, target_point

def animate_controller():
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)

    ax.legend(['Actual Path', 'Measured Path', 'Target'])

    ani = FuncAnimation(fig, update, frames=500, interval=20, blit=True)
    # ani.save('output.mp4', fps=30)

    plt.xlabel('X Pos')
    plt.ylabel('Y Pos')

    plt.show()

# fig, ax = plt.subplots()

# line, = ax.plot([], [])
# measured_line, = ax.plot([], [], 'g--', alpha=0.6)
# target_point, = ax.plot([], [], 'ro', markersize=5)

# animate_controller()

'''
Kp: 0.5 - 5
Kd: 0 - 5
Ki: 0 - 1
alpha: 0.01 - 0.5
'''

best_cost = float('inf')
best_params = None

for _ in range(200):
    Kp = random.uniform(0.5, 5)
    Kd = random.uniform(0, 5)
    Ki = random.uniform(0.0, 0.5)
    alpha = random.uniform(0.01, 0.3)

    cost = mean(simulate(Kp, Kd, Ki, alpha) for i in range(5))

    if cost < best_cost:
        best_cost = cost
        best_params = (Kp, Kd, Ki, alpha)

original_cost = simulate(Kp, Kd, Ki, alpha)

print('Best params:', best_params)
print('Best cost:', best_cost)
print('Original cost:', original_cost)