import numpy as np
import math

L = 0.6                 # Robot width
max_accel = 2           # maximal acceleration permitted for the robot
max_velo = 3            # maximal velocity permitted for the robot
point_num = 100         # the number of points generated along the path


def motion_profiler(x, y, angle):

    # heuristically calculated weights in the path
    w1 = Y
    w2 = math.sqrt(X ** 2 + Y ** 2)
    # the coefficients of the path generated
    Ax = (np.cos(angle) * w2 - 2 * X)
    Ay = (w1 + np.sin(angle) * w2 - 2 * Y)
    Bx = 3 * X - np.cos(angle) * w2
    By = 3 * Y - np.sin(angle) * w2 - 2 * w1
    Cy = w1
    z = np.linspace(0, 1, point_num)

    # path for the robots center
    xline = Ax * z ** 3 + Bx * z ** 2
    yline = Ay * z ** 3 + By * z ** 2 + Cy * z

    # derivatives of the path to calculate heading
    RePrime = 3 * Ax * z ** 2 + 2 * Bx * z
    ImPrime = 3 * Ay * z ** 2 + 2 * By * z + Cy

    # difference in both axes for the right and left motors
    dy = L / 2 * np.sin(math.pi - np.arctan(-1 * RePrime / ImPrime))
    dx = L / 2 * np.cos(math.pi - np.arctan(-1 * RePrime / ImPrime))

    # left and right wheel paths
    l_wheel_x = xline + dx
    l_wheel_y = yline - dy
    r_wheel_x = xline - dx
    r_wheel_y = yline + dy

    # numpy arrays for the motion profile (location, velocity and time)
    l_vel = np.zeros(point_num)
    r_vel = np.zeros(point_num)
    l_time = np.zeros(point_num)
    r_time = np.zeros(point_num)
    l_loc = np.zeros(point_num)
    r_loc = np.zeros(point_num)

    # calculate velocities going forward
    for i in range(1, point_num - 1):
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)
        # calculates right velocity based on the left maximum
        l_vel[i] = np.sqrt(l_vel[i - 1] ** 2 + 2 * max_accel * l_dist)
        r_vel[i] = (l_vel[i - 1] + l_vel[i]) * (r_dist / l_dist) - r_vel[i - 1]
        # enforcing max velocity limitation
        if l_vel[i] > max_velo:
            l_vel[i] = max_velo
            r_vel[i] = (l_vel[i - 1] + l_vel[i]) * (r_dist / l_dist)
            r_vel[i] -= r_vel[i - 1]
        if r_vel[i] > max_velo:
            r_vel[i] = max_velo
            l_vel[i] = (r_vel[i - 1] + r_vel[i]) * (l_dist / r_dist)
            l_vel[i] -= l_vel[i - 1]

    # calculates deacceleration limits
    for i in range(point_num - 1, 1, -1):
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)
        # calculates the max velocity needed for slowing down
        l_vel_max = np.sqrt(l_vel[i] ** 2 + 2 * max_accel * l_dist)
        r_vel_max = np.sqrt(r_vel[i] ** 2 + 2 * max_accel * r_dist)
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
    for i in range(1, point_num):
        l_dist = np.sqrt((l_wheel_x[i] - l_wheel_x[i - 1]) ** 2 +
                         (l_wheel_y[i] - l_wheel_y[i - 1]) ** 2)
        r_dist = np.sqrt((r_wheel_x[i] - r_wheel_x[i - 1]) ** 2 +
                         (r_wheel_y[i] - r_wheel_y[i - 1]) ** 2)
        # calculates left and right times (should be identical)
        r_time[i] = r_time[i - 1] + 2 * r_dist / (r_vel[i] + r_vel[i - 1])
        l_time[i] = l_time[i - 1] + 2 * l_dist / (l_vel[i] + l_vel[i - 1])
        # calculates total distance traveled per side
        l_loc[i] = l_loc[i - 1] + l_dist
        r_loc[i] = r_loc[i - 1] + r_dist
        
    left_profile = zip(l_dist, l_vel, l_time)  # generate profile
    right_profile = zip(r_dist, r_vel, r_time)  # generate profile
    
    return left_profile, right_profile
