# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

VERLET = True
G = 6.67408e-11
MARS_MASS = 6.42e23
MARS_RADIUS = 3.3895e6

EARTH_MASS = 5.972e24
EARTH_RADIUS = 6.371e6

def circ_vel(M, r):
    return np.sqrt(G * M / r)

def esc_vel(M, r):
    sqrt2 = 1.414213562
    return sqrt2 * circ_vel(M, r)

# mass, spring constant, initial position and velocity
m = 1
k = 1
R = MARS_RADIUS
x = np.array([R, 0, 0])
v = np.array([0, 1.42 * circ_vel(MARS_MASS, R), 0])
# print(v[1])
# simulation time, timestep and time
t_max = 100000
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []
a_list = []

def sq_mag(r):
    return r[0] * r[0] + r[1] * r[1] + r[2] * r[2]

def mag(r):
    return np.sqrt(sq_mag(r))

def normalise(r):
    m = mag(r)
    if m == 0:
        return np.zeros(3)
    return r / m

def gravity(M, m, r):
    sq = sq_mag(r)
    if sq == 0:
        return np.zeros(3)
    return -(G * M * m / sq) * normalise(r)

def spring(k, m, dl):
    return -(k / m) * dl

# Euler integration
for i in range(len(t_array)):
    t = t_array[i]
    a = gravity(MARS_MASS, m, x)

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)
    a_list.append(a)

    # calculate new position and velocity
    if VERLET:
        if i == 0:
            # First iteration
            x = x + dt * v
            v = v + dt * a
        else:
            x = 2 * x - x_list[-2] + dt * dt * a
            v = (x - x_list[-2]) / (2 * dt)
    else:
        x = x + dt * v
        v = v + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)
a_array = np.array(a_list)

TRAJECTORIES = True

# plot the position-time graph
plt.figure(1)
plt.clf()
if TRAJECTORIES:
    # Plot trajectories
    plt.plot(x_array[:, 0], x_array[:, 1])
    plt.axis('equal')
else:
    # Plot altitude
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array[:, 0] - MARS_RADIUS, label='x (m)')
    plt.plot(t_array, v_array[:, 0], label='v (m/s)')
    plt.plot(t_array, a_array[:, 0], label='a (m/s^2)')
    plt.legend()
plt.show()
