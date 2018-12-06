import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro', animated=True)

def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,

def update(frame):
    xdata.append(frame[0])
    ydata.append(frame[1])
    ln.set_data(xdata, ydata)
    return ln,

xs = np.linspace(0, 2*np.pi, 128)
ys = [np.sin(x) for x in xs]
ani = FuncAnimation(fig, update, frames= zip(xs, ys),
                    init_func=init, blit=True)
plt.show()
