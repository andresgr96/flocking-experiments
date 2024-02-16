import matplotlib.pyplot as plt
import numpy as np


class SwarmPlotter:

    def __init__(self, n_agents):
        self.n_agents = n_agents
        plt.figure(facecolor='black', edgecolor='black')
        plt.ion()
        plt.show()

    @staticmethod
    def update_plot(pos_xs, pos_ys, pos_hs):
        plt.scatter(pos_xs, pos_ys, color="white", s=15)
        plt.quiver(pos_xs, pos_ys, np.cos(pos_hs), np.sin(pos_hs), color="white", width=0.005, scale=40)
        plt.axis([0, 200, 0, 200])
        plt.gca().set_facecolor('black')
        centroid_x = np.mean(pos_xs)
        centroid_y = np.mean(pos_ys)
        plt.xlim([centroid_x-5, centroid_x+5])
        plt.ylim([centroid_y - 5, centroid_y + 5])
        plt.draw()
        plt.pause(0.0000001)
        plt.clf()