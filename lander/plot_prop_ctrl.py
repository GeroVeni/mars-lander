import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt('proportional_control.log')
time = results[:, 0]
height = results[:, 1]
target = results[:, 2]
actual = results[:, 3]

# plt.figure()
# plt.title('Time - Height')
# plt.xlabel('time (s)')
# plt.ylabel('height (m)')
# plt.grid()
# plt.plot(time, height)

# plt.figure()
# plt.title('Time - Velocities')
# plt.xlabel('time (s)')
# plt.ylabel('velocity (m / s)')
# plt.grid()
# plt.plot(time, actual, label = 'actual')
# plt.plot(time, target, label = 'target')
# plt.legend()

plt.figure()
plt.title('Height - Velocities')
plt.ylabel('height (m)')
plt.xlabel('velocity (m / s)')
plt.grid()
plt.plot(-actual, height, label = 'actual')
plt.plot(-target, height, label = 'target')
plt.legend()

plt.show()
