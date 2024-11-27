from controllers_models import *


timescale = 0.01

x = [0]
y = [120]
theta = [-math.pi/2]
t = [0]
v = []
w = []
milestone_t = []
#(x_ref, y_ref, v_ref, w_ref)
milestone = [(30, 0, 2, 0.1),
              (40, 20, 1, 0),
              (120, 0, 1.5, 0),
              (180, 40, 1, 0),
              (120, 80, 1.7, 0),
              (60, 50, 1, 0)]

i = 0
epsilon = 0.1

car = DriveModel(x[0], y[0], theta[0], timescale)

Kx = 2
Ky = 0.5
Kw = 0.1
max_vel = 5
min_vel = 1
max_ang_vel = math.pi/4
min_ang_vel = -math.pi/4

controller = KanayamaController(Kx, Ky, Kw, max_vel, min_vel, max_ang_vel, min_ang_vel)

for idx in range(100000):
    target = milestone[i]
    v_input, w_input = controller(target[0], target[1], angle_confine(math.atan2(target[1] - y[-1], target[0] - x[-1])), x[-1], y[-1], theta[-1], target[2], target[3])
    x_output, y_output, theta_output = car(v_input, w_input)
    x.append(x_output)
    y.append(y_output)
    v.append(v_input)
    w.append(w_input)
    theta.append(theta_output)
    t.append(t[-1] + timescale)
    distance = ((x[-1] - target[0])**2 + (y[-1] - target[1])**2)**0.5
    if distance < epsilon:
        print(f"reached checkpoint {i}")
        milestone_t.append(t[-1])
        i += 1
    if i == len(milestone):
        break

v = [v[0]] + v
w = [w[0]] + w

import matplotlib.pyplot as plt
plt.subplot(2, 2, 1)
plt.plot(x, y)
milestone_x = [ele[0] for ele in milestone]
milestone_y = [ele[1] for ele in milestone]
plt.scatter(milestone_x, milestone_y, color="red")
plt.title("x, y")

plt.subplot(2, 2, 2)
plt.plot(t, v)
plt.vlines(milestone_t, min(v), max(v), colors="red", linestyles="dashed")
plt.title("v-t")

plt.subplot(2, 2, 3)
plt.plot(t, theta)
plt.vlines(milestone_t, min(theta), max(theta), colors="red", linestyles="dashed")
plt.title("theta-t")

plt.subplot(2, 2, 4)
plt.plot(t, w)
plt.vlines(milestone_t, min(w), max(w), colors="red", linestyles="dashed")
plt.title("w-t")
plt.show()