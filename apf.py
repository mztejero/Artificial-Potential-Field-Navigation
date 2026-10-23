import datetime
import numpy as np
import matplotlib.pyplot as plt

from apf_lib import APF

# fig, ax = plt.subplots()

# ax.set_aspect('equal')
# ax.set_xlim(-20, 20)
# ax.set_ylim(-20, 20)
# ax.grid(True)

# x = np.linspace(-20, 20, 50)
# y = np.linspace(-20, 20, 50)
# X, Y = np.meshgrid(x, y)

# U = -1 / (X - 20) ** 2 + 1 / (X + 20) ** 2
# V = X - X

# ax.quiver(X, Y, U, V, color='orange',
#           angles='xy', scale_units='xy', scale=1)

# plt.show()

apf = APF()

x_array = np.array([])
y_array = np.array([])

while True:
    while not apf.target_found():
        apf.vr, apf.vl = apf.step_pid(apf.v_long, apf.x, apf.y, apf.yaw, apf.params.init.v_ref)
        apf.v_long, apf.x, apf.y, apf.yaw = apf.global_kinematic_model(apf.x, apf.y, apf.yaw, apf.vr, apf.vl)

        x_array = np.append(x_array, apf.x)
        y_array = np.append(y_array, apf.y)

        if len(x_array) > 2:
            x_array = np.delete(x_array, 0)
        if len(y_array) > 2:
            y_array = np.delete(y_array, 0)

        apf.ax.plot(x_array, y_array, color="black")
        apf.plot_car(apf.x, apf.y, apf.yaw)
        plt.pause(apf.params.dt)
    apf.plot_random_target()
    # apf.create_apf()
    apf.plot_apf()