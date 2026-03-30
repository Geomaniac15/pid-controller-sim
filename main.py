import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import cos, sin, sqrt, inf, pi
from statistics import mean
import random
from mpl_toolkits.mplot3d import Axes3D

pos = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]

max_speed = 2.0

centre_x = 1.0
centre_y = 1.0
R = 0.5

Kp = 4.84 # proportional - accelerator
Kd = 3.16 # derivative - brakes
Ki = 0.06

noisy_x_pos = []
noisy_y_pos = []
noisy_z_pos = []

measured_x_pos = []
measured_y_pos = []
measured_z_pos = []

alpha = 0.19
dt = 0.01
limit = 5

# initialise filtered state
filtered_x = 0.0
filtered_y = 0.0
filtered_z = 0.0

# error of sums
error_sum_x = 0.0
error_sum_y = 0.0
error_sum_z = 0.0

def step_system(pos, velocity, filtered_x, filtered_y, filtered_z, error_sum_x, error_sum_y, error_sum_z,
                Kp, Kd, Ki, alpha, t, mass, noise_level, wind_scale):
    # wind
    wind_x = wind_scale * sin(0.7 * t)
    wind_y = -0.5 * wind_scale * cos(0.5 * t)
    wind_z = wind_scale * sin(0.3 * t)

    # drag coefficient
    drag = 0.3

    # target
    x_target = centre_x + R * cos(t) + 0.2 * sin(3*t)
    y_target = centre_y + R * sin(t)
    z_target = 1.0 + 0.5 * sin(0.5 * t)

    # derivatives for feed-forward
    vx_target = -R * sin(t) + 0.6 * cos(3*t)
    vy_target = R * cos(t)
    vz_target = 0.25 * cos(0.5 * t)

    ax_target = -R * cos(t) - 1.8 * sin(3*t)
    ay_target = -R * sin(t)
    az_target = -0.125 * sin(0.5 * t)

    # noise
    measured_x = pos[0] + random.uniform(-noise_level, noise_level)
    measured_y = pos[1] + random.uniform(-noise_level, noise_level)
    measured_z = pos[2] + random.uniform(-noise_level, noise_level)

    # filtering
    filtered_x = alpha * measured_x + (1 - alpha) * filtered_x
    filtered_y = alpha * measured_y + (1 - alpha) * filtered_y
    filtered_z = alpha * measured_z + (1 - alpha) * filtered_z

    # error
    error_x = x_target - filtered_x
    error_y = y_target - filtered_y
    error_z = z_target - filtered_z

    # PID control
    ax = (
        Kp * error_x 
        + Ki * error_sum_x 
        - Kd * (velocity[0] - vx_target)
        )
    
    ay = (
        Kp * error_y 
        + Ki * error_sum_y 
        - Kd * (velocity[1] - vy_target)
        )
    
    az = (
        Kp * error_z
        + Ki * error_sum_z
        - Kd * (velocity[2] - vz_target)
    )
    
    ax += ax_target
    ay += ay_target
    az += az_target

    # drag force (opposes current velocity)
    drag_x = -drag * velocity[0]
    drag_y = -drag * velocity[1]
    drag_z = -drag * velocity[2]

    # physics update
    velocity[0] += (ax + wind_x + drag_x) / mass * dt
    velocity[1] += (ay + wind_y + drag_y) / mass * dt
    velocity[2] += (az + wind_z + drag_z) / mass * dt

    speed = sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
    if speed > max_speed:
        scale = max_speed / speed
        velocity[0] *= scale
        velocity[1] *= scale
        velocity[2] *= scale

    pos[0] += velocity[0] * dt
    pos[1] += velocity[1] * dt
    pos[2] += velocity[2] * dt

    # integral update
    error_sum_x += error_x * dt
    error_sum_y += error_y * dt
    error_sum_z += error_z * dt

    error_sum_x = max(min(error_sum_x, limit), -limit)
    error_sum_y = max(min(error_sum_y, limit), -limit)
    error_sum_z = max(min(error_sum_z, limit), -limit)

    ax_total = ax + wind_x + drag_x
    ay_total = ay + wind_y + drag_y
    az_total = az + wind_z + drag_z

    return (
        pos, velocity,
        filtered_x, filtered_y, filtered_z,
        error_sum_x, error_sum_y, error_sum_z,
        x_target, y_target, z_target,
        measured_x, measured_y, measured_z,
        ax_total, ay_total, az_total,
        wind_x, wind_y, wind_z,
        ax, ay, az,
        drag_x, drag_y, drag_z,
    )

def simulate(Kp, Kd, Ki=0.0, alpha=0.1):
    # reset state for each simulation
    pos = [0.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0]
    filtered_x = 0.0
    filtered_y = 0.0
    filtered_z = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0
    error_sum_z = 0.0

    # randomise world parameters
    mass = random.uniform(2.0, 5.0)
    noise_level = random.uniform(0.1, 0.4)
    wind_scale = random.uniform(0.2, 0.6)

    total_cost = 0.0

    on_target_steps = 0
    success = False
    timestep_found = 0

    for step in range(1000):
        t = step * dt

        (
            pos, velocity,
            filtered_x, filtered_y, filtered_z,
            error_sum_x, error_sum_y, error_sum_z,
            x_target, y_target, z_target,
            measured_x, measured_y, measured_z,
            ax_total, ay_total, az_total,
            wind_x, wind_y, wind_z,
            ax, ay, az,
            drag_x, drag_y, drag_z,
        ) = step_system(
            pos, velocity, filtered_x, filtered_y, filtered_z,
            error_sum_x, error_sum_y, error_sum_z,
            Kp, Kd, Ki, alpha, t, mass, noise_level, wind_scale
        )

        # rewards closeness to target, quick reactions, low lag
        ex = x_target - pos[0]
        ey = y_target - pos[1]
        ez = z_target - pos[2]

        total_cost += (ex*ex + ey*ey + 0.5 * ez*ez)

        # rewards slower movements, smoothness, low aggressive acceleration
        total_cost += 0.05 * (
            velocity[0]**2 + velocity[1]**2 + velocity[2]**2
        )

        # penalises aggressive control inputs
        total_cost += 0.01 * (
            ax_total**2 + ay_total**2 + az_total**2
        )

        # success conditions
        distance_sq = (
            (x_target - pos[0])**2 + 
            (y_target - pos[1])**2 +
            (z_target - pos[2])**2
        )
        
        if distance_sq < 0.2**2:
            on_target_steps += 1
        else:
            on_target_steps = 0

        if on_target_steps > 50:
            success = True
            timestep_found = step * dt

    # apply failure penalty once
    if not success:
        total_cost += 500

    return total_cost, success, timestep_found

def update(frame):
    t = (frame * dt) % (2 * pi)

    global filtered_x, filtered_y, filtered_z, error_sum_x, error_sum_y, error_sum_z

    mass = 3.0
    noise_level = 0.2
    wind_scale = 0.4

    (
        pos[:], velocity[:],
        filtered_x, filtered_y, filtered_z,
        error_sum_x, error_sum_y, error_sum_z,
        x_target, y_target, z_target,
        measured_x, measured_y, measured_z,
        ax_total, ay_total, az_total,
        wind_x, wind_y, wind_z,
        ax, ay, az,
        drag_x, drag_y, drag_z,
    ) = step_system(
        pos, velocity, filtered_x, filtered_y, filtered_z,
        error_sum_x, error_sum_y, error_sum_z,
        Kp, Kd, Ki, alpha, t, mass, noise_level, wind_scale
    )

    noisy_x_pos.append(pos[0])
    noisy_y_pos.append(pos[1])
    noisy_z_pos.append(pos[2])

    measured_x_pos.append(measured_x)
    measured_y_pos.append(measured_y)
    measured_z_pos.append(measured_z)

    max_points = 2000
    if len(noisy_x_pos) > max_points:
        noisy_x_pos.pop(0)
        noisy_y_pos.pop(0)
        noisy_z_pos.pop(0)
        measured_x_pos.pop(0)
        measured_y_pos.pop(0)
        measured_z_pos.pop(0)

    line.set_data(noisy_x_pos, noisy_y_pos)
    line.set_3d_properties(noisy_z_pos)

    measured_line.set_data(measured_x_pos, measured_y_pos)
    measured_line.set_3d_properties(measured_z_pos)

    target_point.set_data([x_target], [y_target])
    target_point.set_3d_properties([z_target])

    return line, measured_line, target_point

def animate_controller():
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)
    ax.set_zlim(0, 3)

    ax.legend()

    ani = FuncAnimation(fig, update, frames=int(2*pi / dt), interval=20, blit=False)
    ani.save('demo.gif', writer='pillow', fps=30)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

def optimise():
    best_cost = float('inf')
    best_params = None
    best_success = False
    best_timestep = 0

    for _ in range(200):
        Kp = random.uniform(0.5, 5)
        Kd = random.uniform(0, 5)
        Ki = random.uniform(0.0, 0.5)
        alpha = random.uniform(0.01, 0.3)

        results = [simulate(Kp, Kd, Ki, alpha) for _ in range(5)]

        # separate values
        costs = [r[0] for r in results]
        successes = [r[1] for r in results]
        timesteps = [r[2] for r in results]

        cost = mean(costs)
        success = any(successes)
        timestep_found = min([t for t in timesteps if t > 0], default=0)

        if cost < best_cost:
            best_cost = cost
            best_params = (Kp, Kd, Ki, alpha)
            best_success = success
            best_timestep = timestep_found

    print('Best params')
    print(f'Kp: {best_params[0]}, Kd: {best_params[1]}, Ki: {best_params[2]}, alpha: {best_params[3]}')
    print('Best cost:', best_cost)

    print(f'Success: {best_success}')
    print(f'Found at timestep {best_timestep}')

    return best_params

def run():
    global fig, ax, line, measured_line, target_point

    # reset state before each run
    global pos, velocity
    global filtered_x, filtered_y, filtered_z
    global error_sum_x, error_sum_y, error_sum_z

    pos = [0.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0]

    filtered_x = filtered_y = filtered_z = 0.0
    error_sum_x = error_sum_y = error_sum_z = 0.0

    noisy_x_pos.clear()
    noisy_y_pos.clear()
    noisy_z_pos.clear()

    measured_x_pos.clear()
    measured_y_pos.clear()
    measured_z_pos.clear()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    line, = ax.plot([], [], [], label='Actual Path')
    measured_line, = ax.plot([], [], [], 'g--', alpha=0.6, label='Measured Path')
    target_point, = ax.plot([], [], [], 'ro', markersize=5, label='Target')

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