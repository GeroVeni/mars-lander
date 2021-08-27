# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# control constants
SHOW_MANUAL = False

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []
x_list_verlet = []
v_list_verlet = []

# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x / m
    x = x + dt * v
    v = v + dt * a

# Re-initialize position and velocity
x_prev = x = 0
v = 1
x_list_verlet.append(x)
x = x + dt * v

# Verlet integration
for t in t_array:

	# append current state to trajectories
	x_list_verlet.append(x)
	v_list_verlet.append(v)

	# calculate new position and velocity
	a = - k * x / m
	x_new = 2 * x - x_prev + dt**2 * a
	v = (x_new - x_prev) / (2 * dt)
	x_prev = x
	x = x_new


# convert trajectory lists into arrays, so they can be indexed more easily
x_array = np.array(x_list)
v_array = np.array(v_list)
x_array_verlet = np.array(x_list_verlet[:-1])
v_array_verlet = np.array(v_list_verlet)

x_array_manual = np.sin(t_array)
v_array_manual = np.cos(t_array)

# plot the position-time graph
f, axarr = plt.subplots(2, sharex = True)
plt.xlabel('time (s)')
for ax in axarr: ax.grid()

# plot euler
axarr[0].set_title('Euler integration')
axarr[0].plot(t_array, x_array, 'b', label='x (m)')
axarr[0].plot(t_array, v_array, 'g', label='v (m/s)')

#plot verlet
axarr[1].set_title('Verlet integration')
axarr[1].plot(t_array, x_array_verlet, 'b', label='x (m) - Verlet')
axarr[1].plot(t_array, v_array_verlet, 'g', label='v (m/s) - Verlet')

#plot manually calculated solution
if SHOW_MANUAL:
	for ax in axarr:
		ax.plot(t_array, x_array_manual, 'b--', label='x (m) - Manual')
		ax.plot(t_array, v_array_manual, 'g--', label='v (m/s) - Manual')

for ax in axarr: ax.legend()
plt.show()
