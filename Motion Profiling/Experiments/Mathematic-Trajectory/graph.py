from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import math

X = -0.4
Y = 1.8
L = 0.6
max_accel = 2
max_velo = 3
point_num = 100

angle = math.pi / 3
w1 = Y
w2 = math.sqrt(X ** 2 + Y ** 2)
Ax = (np.cos(angle) * w2 - 2 * X)
Ay = (w1 + np.sin(angle) * w2 - 2 * Y)
Bx = 3 * X - np.cos(angle) * w2
By = 3 * Y - np.sin(angle) * w2 - 2 * w1
Cy = w1

axes = plt.axes(projection='3d')

axes.set_aspect('equal')
# Data for a three-dimensional line
z = np.linspace(0, 1, point_num)
xline = Ax * z ** 3 + Bx * z ** 2
yline = Ay * z ** 3 + By * z ** 2 + Cy * z
RePrime = 3 * Ax * z ** 2 + 2 * Bx * z
ImPrime = 3 * Ay * z ** 2 + 2 * By * z + Cy
dy = L/2 * np.sin(math.pi - np.arctan(-1 * RePrime / ImPrime))
dx = L/2 * np.cos(math.pi - np.arctan(-1 * RePrime / ImPrime))

l_vel = np.zeros(point_num)
r_vel = np.zeros(point_num)
l_time = np.zeros(point_num)
r_time = np.zeros(point_num)

for i in range(1, point_num-1):
    l_dist = np.sqrt((xline[i] + dx[i] - xline[i - 1] - dx[i - 1]) ** 2 + (yline[i] - dy[i] - yline[i - 1] + dy[i - 1]) ** 2)
    r_dist = np.sqrt((xline[i] - dx[i] - xline[i - 1] + dx[i - 1]) ** 2 + (yline[i] + dy[i] - yline[i - 1] - dy[i - 1]) ** 2)
    l_vel[i] = np.sqrt(l_vel[i-1]**2 + 2 * max_accel * l_dist)
    r_vel[i] = (l_vel[i-1] + l_vel[i]) * (r_dist / l_dist) - r_vel[i-1]
    if l_vel[i] > max_velo:
        l_vel[i] = max_velo
        r_vel[i] = (l_vel[i-1] + l_vel[i]) * (r_dist / l_dist) - r_vel[i-1]
    if r_vel[i] > max_velo:
        r_vel[i] = max_velo
        l_vel[i] = (r_vel[i-1] + r_vel[i]) * (l_dist / r_dist) - l_vel[i-1]

for i in range(point_num-1, 1, -1):
    l_dist = np.sqrt((xline[i] + dx[i] - xline[i - 1] - dx[i - 1]) ** 2 + (yline[i] - dy[i] - yline[i - 1] + dy[i - 1]) ** 2)
    r_dist = np.sqrt((xline[i] - dx[i] - xline[i - 1] + dx[i - 1]) ** 2 + (yline[i] + dy[i] - yline[i - 1] - dy[i - 1]) ** 2)
    l_vel_max = np.sqrt(l_vel[i]**2 + 2 * max_accel * l_dist)
    r_vel_max = np.sqrt(r_vel[i] ** 2 + 2 * max_accel * r_dist)
    if l_vel[i-1] > l_vel_max:
        l_vel[i-1] = l_vel_max
        r_vel[i-1] = (l_vel[i-1] + l_vel[i]) * (r_dist / l_dist) - r_vel[i]
    if r_vel[i-1] > r_vel_max:
        r_vel[i-1] = r_vel_max
        l_vel[i-1] = (r_vel[i-1] + r_vel[i]) * (l_dist / r_dist) - l_vel[i]

for i in range(1, point_num):
    l_dist = np.sqrt((xline[i] + dx[i] - xline[i - 1] - dx[i - 1]) ** 2 + (yline[i] - dy[i] - yline[i - 1] + dy[i - 1]) ** 2)
    r_dist = np.sqrt((xline[i] - dx[i] - xline[i - 1] + dx[i - 1]) ** 2 + (yline[i] + dy[i] - yline[i - 1] - dy[i - 1]) ** 2)
    r_time[i] = r_time[i-1] + 2 * r_dist / (r_vel[i] + r_vel[i-1])
    l_time[i] = l_time[i-1] + 2 * l_dist / (l_vel[i] + l_vel[i-1])
    print r_time[i] - l_time[i]

axes.scatter(xline, yline, z)
axes.scatter(xline + dx, yline - dy, z, c=l_vel, cmap=plt.spring())
axes.scatter(xline - dx, yline + dy, z, c=r_vel, cmap=plt.spring())

plt.show()
