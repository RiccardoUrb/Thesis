import matplotlib.pyplot as plt
import matplotlib.animation as anim
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import main
# import re
import time
# import pickle
# from matplotlib.colors import ListedColormap

class AnimatedPlot(object):
    def __init__(self, map_object, duration):
        self.fig = plt.figure(dpi=150)
        self.ax = p3.Axes3D(self.fig)
        self.stream = self.data_stream()
        self.drone = None
        self.ax.set_yticks([])
        self.ax.set_xticks([])
        self.ax.set_zticks([])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # self.ax.set_xlim3d([0.0, map_object.x_dimension])
        # self.ax.set_ylim3d([0.0, map_object.y_dimension])
        # self.ax.set_zlim3d([0.0, map_object.z_dimension])
        self.ani = anim.FuncAnimation(self.fig, self.update,
                                           init_func=self.setup_plot, frames=(duration * 5), interval=50, blit=True)

    def setup_plot(self):
        """Initial drawing of the scatter plot."""
        x, y, z = next(self.stream).T
        self.drone = self.ax.scatter(x, y, c='red', s=40, edgecolor="k")
        return self.drone,

    def data_stream(self):
        """Generate a random walk (brownian motion). Data is scaled to produce
        a soft "flickering" effect."""
        while True:
            drone_position = np.random.random([1, 3])
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            yield np.c_[drone_position]

    def update(self, i):
        """Update the scatter plot."""
        data = next(self.stream)
        # drone_position = data
        # self.drone.set_sizes(300 * abs(data[:, 2])**1.5 + 100)
        # self.drone.set_offsets(data[:, :2])
        self.drone.set_3d_properties(data[:, 2], zdir='z')
        # We need to return the updated artist for FuncAnimation to draw..
        # Note that it expects a sequence of artists, thus the trailing comma.
        return self.drone,


if __name__ == '__main__':
    a = AnimatedPlot(None, 10)
    plt.show()