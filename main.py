import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import cos, sin, sqrt, inf, pi
from statistics import mean
import random

pos = [0.0, 0.0]
velocity = [0.0, 0.0]

max_speed = 2.0

centre_x = 2.0
centre_y = 1.0
R = 0.5

Kp = 4.84 # proportional - accelerator
Kd = 3.16 # derivative - brakes
Ki = 0.06

noisy_x_pos = []
noisy_y_pos = []
measured_x_pos = []
measured_y_pos = []

alpha = 0.19
dt = 0.01
limit = 5

# initialise filtered state
filtered_x = 0.0
filtered_y = 0.0

# error of sums
error_sum_x = 0.0
error_sum_y = 0.0

def step_system(pos, velocity, filtered_x, filtered_y, error_sum_x, error_sum_y,
                Kp, Kd, Ki, alpha, t):
    # wind
    wind_x = 0.4 * sin(0.7 * t)
    wind_y = -0.2 * cos(0.5 * t)

    # drag coefficient
    drag = 0.3

    # target
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

    # PID control
    ax = Kp * error_x + Ki * error_sum_x - Kd * velocity[0]
    ay = Kp * error_y + Ki * error_sum_y - Kd * velocity[1]

    # drag force (opposes current velocity)
    drag_x = -drag * velocity[0]
    drag_y = -drag * velocity[1]

    # physics update
    velocity[0] += (ax + wind_x + drag_x) * dt
    velocity[1] += (ay + wind_y + drag_y) * dt

    speed = sqrt(velocity[0]**2 + velocity[1]**2)
    if speed > max_speed:
        scale = max_speed / speed
        velocity[0] *= scale
        velocity[1] *= scale

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt

    # integral update
    error_sum_x += error_x * dt
    error_sum_y += error_y * dt

    error_sum_x = max(min(error_sum_x, limit), -limit)
    error_sum_y = max(min(error_sum_y, limit), -limit)

    ax_total = ax + wind_x + drag_x
    ay_total = ay + wind_y + drag_y

    return pos, velocity, filtered_x, filtered_y, error_sum_x, error_sum_y, x_target, y_target, measured_x, measured_y, ax_total, ay_total, wind_x, wind_y, ax, drag_x, drag_y

def simulate(Kp, Kd, Ki=0.0, alpha=0.1):
    # reset state for each simulation
    pos = [0.0, 0.0]
    velocity = [0.0, 0.0]
    filtered_x = 0.0
    filtered_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0

    total_cost = 0.0

    for step in range(1000):
        t = step * dt

        pos, velocity, filtered_x, filtered_y, error_sum_x, error_sum_y, x_target, y_target, *_ = step_system(
            pos, velocity, filtered_x, filtered_y, error_sum_x, error_sum_y,
            Kp, Kd, Ki, alpha, t
        )

        total_cost += (x_target - pos[0])**2 + (y_target - pos[1])**2

    return total_cost

def update(frame):
    t = (frame * dt) % (2 * pi)

    global filtered_x, filtered_y, error_sum_x, error_sum_y

    pos[:], velocity[:], filtered_x, filtered_y, error_sum_x, error_sum_y, x_target, y_target, measured_x, measured_y, ax_total, ay_total, wind_x, wind_y, ax, drag_x, drag_y = step_system(
        pos, velocity, filtered_x, filtered_y, error_sum_x, error_sum_y,
        Kp, Kd, Ki, alpha, t
    )

    noisy_x_pos.append(pos[0])
    noisy_y_pos.append(pos[1])
    measured_x_pos.append(measured_x)
    measured_y_pos.append(measured_y)

    line.set_data(noisy_x_pos, noisy_y_pos)
    measured_line.set_data(measured_x_pos, measured_y_pos)
    target_point.set_data([x_target], [y_target])

    wind_vec.set_offsets([pos[0], pos[1]])
    wind_vec.set_UVC(wind_x, wind_y)

    control_vec.set_offsets([pos[0], pos[1]])
    control_vec.set_UVC(ax_total, ay_total)

    drag_vec.set_offsets([pos[0], pos[1]])
    drag_vec.set_UVC(drag_x, drag_y)

    return line, measured_line, target_point, wind_vec, control_vec

def animate_controller():
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)

    ax.legend(['Actual Path', 'Measured Path', 'Target'])

    ani = FuncAnimation(fig, update, frames=int(2*pi / dt), interval=20, blit=False)
    # ani.save('output.mp4', fps=30)

    plt.xlabel('X Pos')
    plt.ylabel('Y Pos')

    plt.show()

def optimise():
    best_cost = float('inf')
    best_params = None

    for _ in range(200):
        Kp = random.uniform(0.5, 5)
        Kd = random.uniform(0, 5)
        Ki = random.uniform(0.0, 0.5)
        alpha = random.uniform(0.01, 0.3)

        cost = mean(simulate(Kp, Kd, Ki, alpha) for _ in range(5))

        if cost < best_cost:
            best_cost = cost
            best_params = (Kp, Kd, Ki, alpha)

    print('Best params:', best_params)
    print('Best cost:', best_cost)

    return best_params

def run():
    global fig, ax, line, measured_line, target_point
    global wind_vec, control_vec, drag_vec

    fig, ax = plt.subplots()

    line, = ax.plot([], [])
    measured_line, = ax.plot([], [], 'g--', alpha=0.6)
    target_point, = ax.plot([], [], 'ro', markersize=5)

    wind_vec = ax.quiver(0, 0, 0, 0, color='blue', scale=5)
    control_vec = ax.quiver(0, 0, 0, 0, color='green', scale=5)
    drag_vec = ax.quiver(0, 0, 0, 0, color='red', scale=5)

    animate_controller()

if __name__ == '__main__':
    MODE = 'optimise'  # options: 'run' or 'optimise'

    if MODE == 'optimise':
        params = optimise()
        if params:
            Kp, Kd, Ki, alpha = params
            print('\nRunning with optimised parameters...\n')
            run()
    else:
        run()

'''
Kp: 0.5 - 5
Kd: 0 - 5
Ki: 0 - 1
alpha: 0.01 - 0.5
'''

# best_cost = float('inf')
# best_params = None

# for _ in range(200):
#     Kp = random.uniform(0.5, 5)
#     Kd = random.uniform(0, 5)
#     Ki = random.uniform(0.0, 0.5)
#     alpha = random.uniform(0.01, 0.3)

#     cost = mean(simulate(Kp, Kd, Ki, alpha) for i in range(5))

#     if cost < best_cost:
#         best_cost = cost
#         best_params = (Kp, Kd, Ki, alpha)

# # original_cost = simulate(Kp, Kd, Ki, alpha)

# print('Best params:', best_params)
# print('Best cost:', best_cost)
# print('Original cost:', original_cost)