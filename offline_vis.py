import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

r = 3.
t = np.linspace(0, 2*np.pi, 1000)
x_path = np.sin(t) * r
y_path = r + np.cos(t) * r


def animate(i):
    x_pos = []
    y_pos = []
    with open("report.txt", "r") as report_file:
        for line in report_file:
            words = line.split()
            x_pos.append(float(words[0]))
            y_pos.append(float(words[1]))
        psi = float(words[2])
        delta = float(words[3])
    report_file.close()
    ax = plt.axes()
    ax.cla()
    ax.scatter(x_pos, y_pos, s = 1.5, c = 'b', label = 'actual')
    ax.plot(x_path, y_path, c = 'r', label = 'desired')
    ax.set_xlabel('Position x (m)')
    ax.set_ylabel('Position y (m)')
    ax.set_aspect('equal')

ani = FuncAnimation(plt.gcf(), animate, interval=0.1, repeat=False)
plt.axes().legend()
plt.show()
plt.close()