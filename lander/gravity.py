# Gravity simulation in 3 dimensions

import numpy as np
import matplotlib.pyplot as plt
from copy import copy

# Utility functions
mag = lambda x : np.sqrt(x.dot(x))

def validateData(ar):
    crash = False
    for i in range(len(ar)):
        if ar[i] <= 0 or crash:
            ar[i] = 0
            crash = True

def splitInComponents(l):
    x_list = np.array([x for (x, y, z) in l])
    y_list = np.array([y for (x, y, z) in l])
    z_list = np.array([z for (x, y, z) in l])
    return x_list, y_list, z_list

# Initialize state variables
pos = np.array([6.e6, 0. ,0.])
vel = np.array([0., 0., 0.])

# Constants
G = 6.67408e-11		# Universal gravitational constant
M = 6.42e23			# Mass of Mars
m = 1
vel_circular = np.array([0., np.sqrt(G * M / mag(pos)), 0.])
vel_escape = vel_circular * np.sqrt(2)

# simulation time, timestep and time
t_max = 20000.
dt = 20
t_array = np.arange(0, t_max, dt)

# Integrating functions
def euler_integration(pos_init, vel_init):
    
    pos = copy(pos_init)
    vel = copy(vel_init)
    pos_list = []
    vel_list = []

    for t in t_array:

        # append state to lists
        pos_list.append(copy(pos))
        vel_list.append(copy(vel))

        #calculate new position and velocity
        a = -1 * (G * M / mag(pos)**3) * pos
        pos += dt * vel
        vel += dt * a

    return (pos_list, vel_list)

def verlet_integration(pos_init, vel_init):

    pos = copy(pos_init)
    vel = copy(vel_init)
    pos_list = []
    vel_list = []

    pos_list.append(copy(pos))
    pos += dt * vel

    for t in t_array:

        # append state to lists
        pos_list.append(copy(pos))
        vel_list.append(copy(vel))

        # calculate new position and velocity
        a = -1 * (G * M / mag(pos)**3) * pos
        pos = 2 * pos - pos_list[-2] + dt**2 * a
        vel = (pos - pos_list[-2]) / (2 * dt)

    return (pos_list[:-1], vel_list)

def trajectories(pos_init, vel_init):
    pos_list_euler, vel_list_euler = euler_integration(pos_init, vel_init)
    pos_list_verlet, vel_list_verlet = verlet_integration(pos_init, vel_init)
    x_euler, y_euler, z_euler = splitInComponents(pos_list_euler)
    x_verlet, y_verlet, z_verlet = splitInComponents(pos_list_verlet)
    return x_euler, y_euler, x_verlet, y_verlet

# create 4 plots
f, axarr = plt.subplots(2, 2)

# enable grid
for col in axarr:
    for i in col: i.grid()

axis_limits = (-7., 7.)
for i in range(len(axarr)):
    for j in range(len(axarr[i])):
        if i == 0 and j == 0:
            axarr[i][j].set_xlim(0., 3600.)
            axarr[i][j].set_ylim(0., 7.)
            axarr[i][j].set_xlabel('t (s)')
            axarr[i][j].set_ylabel('x (1e6 * m)')
        else:
            axarr[i][j].set_ylim(axis_limits)
            axarr[i][j].set_xlim(axis_limits)
            axarr[i][j].axis('equal')
            axarr[i][j].set_xlabel('x (1e6 * m)')
            axarr[i][j].set_ylabel('y (1e6 * m)')

## Scenario 1
ax = axarr[0][0]
ax.set_title('Scenario 1')
# Record trajectories
(pos_list_euler, vel_list_euler) = euler_integration(pos, vel)
(pos_list_verlet, vel_list_verlet) = verlet_integration(pos, vel)
# Split in components
x_euler = splitInComponents(pos_list_euler)[0]
x_verlet = splitInComponents(pos_list_verlet)[0]
# Validate trajectories
validateData(x_euler)
validateData(x_verlet)
# Plot data
ax.plot(t_array, x_euler / 1e6, label = 'euler')
ax.plot(t_array, x_verlet / 1e6, label = 'verlet')

# Scenario 2
ax = axarr[0][1]
ax.set_title('Scenario 2')
# Assign initiall velocity
vel = vel_circular
# Calculate trajectories
x_euler, y_euler, x_verlet, y_verlet = trajectories(pos, vel)
# Plot trajectories
ax.plot(x_euler / 1e6, y_euler / 1e6, label = 'euler')
ax.plot(x_verlet / 1e6, y_verlet / 1e6, label = 'verlet')

# Scenario 3
ax = axarr[1][0]
ax.set_title('Scenario 3')
# Assign initiall velocity
vel = .8 * vel_circular
# Calculate trajectories
x_euler, y_euler, x_verlet, y_verlet = trajectories(pos, vel)
# Plot trajectories
ax.plot(x_euler / 1e6, y_euler / 1e6, label = 'euler')
ax.plot(x_verlet / 1e6, y_verlet / 1e6, label = 'verlet')

# Scenario 4
ax = axarr[1][1]
ax.set_title('Scenario 4')
# Assign initiall velocity
vel = vel_escape
# Calculate trajectories
x_euler, y_euler, x_verlet, y_verlet = trajectories(pos, vel)
# Plot trajectories
ax.plot(x_euler / 1e6, y_euler / 1e6, label = 'euler')
ax.plot(x_verlet / 1e6, y_verlet / 1e6, label = 'verlet')

for col in axarr: 
    for i in col: i.legend()
f.subplots_adjust(hspace = .5)
f.subplots_adjust(wspace = .5)
plt.show()
