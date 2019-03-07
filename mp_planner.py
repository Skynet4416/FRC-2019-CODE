import numpy as np
import math

WIDTH = 0.6                 # Robot's width
MAX_ACCEL = 2               # maximal acceleration permitted for the robot
MAX_VEL = 3                 # maximal velocity permitted for the robot
POINTS_COUNT = 100          # the number of points generated along the path


def motion_profiler(x, y, angle):
    # heuristically calculated weights in the path
    w1 = y
    w2 = math.sqrt(x ** 2 + y ** 2)
    # the coefficients of the path generated
    a_x = (np.cos(angle) * w2 - 2 * x)
    a_y = (w1 + np.sin(angle) * w2 - 2 * y)
    b_x = 3 * x - np.cos(angle) * w2
    b_y = 3 * y - np.sin(angle) * w2 - 2 * w1
    c_y = w1
    z = np.linspace(0, 1, POINTS_COUNT)

    # path for the robots center
    xline = a_x * z ** 3 + b_x * z ** 2
    yline = a_y * z ** 3 + b_y * z ** 2 + c_y * z

    # Real and imaginary derivatives of the path to calculate heading
    real_d = 3 * a_x * z ** 2 + 2 * b_x * z
    imaginary_d = 3 * a_y * z ** 2 + 2 * b_y * z + c_y

    # difference in both axes for the right and left motors
    dy = WIDTH / 2 * np.sin(math.pi - np.arctan(-1 * real_d / imaginary_d))
    dx = WIDTH / 2 * np.cos(math.pi - np.arctan(-1 * real_d / imaginary_d))

    # left and right wheel paths
    l_wheel_x = xline + dx
    l_wheel_y = yline - dy
    r_wheel_x = xline - dx
    r_wheel_y = yline + dy

    # numpy arrays for the motion profile (location, velocity and time)
    l_loc = np.zeros(POINTS_COUNT)
    r_loc = np.zeros(POINTS_COUNT)
    l_vel = np.zeros(POINTS_COUNT)
    r_vel = np.zeros(POINTS_COUNT)
    l_time = np.zeros(POINTS_COUNT)
    r_time = np.zeros(POINTS_COUNT)

    # calculate velocities going forward
    for i in range(1, POINTS_COUNT - 1):
        # Calculates distances
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)

        # calculates right velocity based on the left maximum
        l_vel[i] = np.sqrt(l_vel[i - 1] ** 2 + 2 * MAX_ACCEL * l_dist)
        r_vel[i] = (l_vel[i - 1] + l_vel[i]) * (r_dist / l_dist) - r_vel[i - 1]

        # enforcing max velocity limitation
        if l_vel[i] > MAX_VEL:
            l_vel[i] = MAX_VEL
            r_vel[i] = (l_vel[i - 1] + l_vel[i]) * (r_dist / l_dist)
            r_vel[i] -= r_vel[i - 1]
        if r_vel[i] > MAX_VEL:
            r_vel[i] = MAX_VEL
            l_vel[i] = (r_vel[i - 1] + r_vel[i]) * (l_dist / r_dist)
            l_vel[i] -= l_vel[i - 1]

    # Fixes velocities by deacceleration limits
    for i in range(POINTS_COUNT - 1, 1, -1):
        # Calculates distances
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)

        # calculates the max velocity needed for slowing down
        l_vel_max = np.sqrt(l_vel[i] ** 2 + 2 * MAX_ACCEL * l_dist)
        r_vel_max = np.sqrt(r_vel[i] ** 2 + 2 * MAX_ACCEL * r_dist)

        # recalculates velocity to match deacceleration limits
        if l_vel[i - 1] > l_vel_max:
            l_vel[i - 1] = l_vel_max
            r_vel[i - 1] = (l_vel[i - 1] + l_vel[i]) * (r_dist / l_dist)
            r_vel[i - 1] -= r_vel[i]
        if r_vel[i - 1] > r_vel_max:
            r_vel[i - 1] = r_vel_max
            l_vel[i - 1] = (r_vel[i - 1] + r_vel[i]) * (l_dist / r_dist)
            l_vel[i - 1] -= l_vel[i]

    # calculates final time and distance per point
    for i in range(1, POINTS_COUNT):
        # Calculates distances
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)

        # calculates left and right times (should be identical)
        r_time[i] = 2 * r_dist / (r_vel[i] + r_vel[i - 1])
        l_time[i] = 2 * l_dist / (l_vel[i] + l_vel[i - 1])

        # calculates total distance traveled per side
        l_loc[i] = l_loc[i - 1] + l_dist
        r_loc[i] = r_loc[i - 1] + r_dist

    # generate profiles
    left_profile = zip(list(l_loc), list(l_vel), list(l_time))
    right_profile = zip(list(r_loc), list(r_vel), list(r_time))

    return list(left_profile), list(right_profile)
