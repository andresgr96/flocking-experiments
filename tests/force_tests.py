import msvcrt

import numpy as np
import time
import plot_swarm_old as plot_swarm


def wrap_to_pi(x):
    x = x % (3.1415926 * 2)
    x = (x + (3.1415926 * 2)) % (3.1415926 * 2)

    x[x > 3.1415926] = x[x > 3.1415926] - (3.1415926 * 2)
    return x


# Constants
epsilon = 12.0
sigma_const = 0.7
umax_const = 0.1
wmax = 1.5708
alpha = 2.0
beta = 0.5
k1 = 0.6
k2 = 0.05
sensing_range = 2.0


def simulate_swarm(n_agents, initial_positions, initial_headings, run_wall_time=1000, dt=0.05, if_plot=True):
    n_agent = n_agents
    pos_xs = initial_positions[:, 0]
    pos_ys = initial_positions[:, 1]
    pos_hs = initial_headings
    sigmas = np.full(n_agent, sigma_const)

    if if_plot:
        plotter = plot_swarm.SwarmPlotter(n_agent)

    for i in range(int(int(run_wall_time / dt))):
        xx1, xx2 = np.meshgrid(pos_xs, pos_xs)
        yy1, yy2 = np.meshgrid(pos_ys, pos_ys)
        d_ij_x = xx1 - xx2
        d_ij_y = yy1 - yy2
        d_ij = np.sqrt(np.multiply(d_ij_x, d_ij_x) + np.multiply(d_ij_y, d_ij_y))
        print(f"Dists: {d_ij[0][1]}")

        d_ij[d_ij > sensing_range] = np.inf
        d_ij[d_ij == 0.0] = np.inf
        ij_ang = np.arctan2(d_ij_y, d_ij_x) - pos_hs[:, np.newaxis]
        print(f"Angles: {ij_ang}")

        forces = -epsilon * (2 * (sigmas[:, np.newaxis] ** 4 / d_ij ** 5) - (sigmas[:, np.newaxis] ** 2 / d_ij ** 3))
        forces[d_ij == np.inf] = 0.0
        print(f"Forces: {forces}")

        alignment_coss = np.sum(np.cos(pos_hs))
        alignment_sins = np.sum(np.sin(pos_hs))
        alignment_angs = np.arctan2(alignment_sins, alignment_coss)
        alignment_mags = np.sqrt(alignment_coss**2 + alignment_sins**2)

        forces_alignment_x = alignment_mags * np.cos(alignment_angs - pos_hs)
        forces_alignment_y = alignment_mags * np.sin(alignment_angs - pos_hs)

        f_x = alpha * np.sum(np.multiply(forces, np.cos(ij_ang)), axis=1)  # + \
              #int(True) * beta * forces_alignment_x
        print(f"Fx: {f_x}")

        f_y = alpha * np.sum(np.multiply(forces, np.sin(ij_ang)), axis=1) #+ \
              #int(True) * beta * forces_alignment_y

        print(f"Fy: {f_y}")

        u = k1 * f_x + 0.05
        u[u > umax_const] = umax_const
        u[u < 0] = 0.0

        w = k2 * f_y
        w[w > wmax] = wmax
        w[w < -wmax] = -wmax

        final_xs = np.multiply(u, np.cos(pos_hs))
        final_ys = np.multiply(u, np.cos(pos_hs))
        pos_xs = pos_xs + final_xs * dt
        pos_ys = pos_ys + final_ys * dt
        pos_hs = wrap_to_pi(pos_hs + w * dt)

        if (not (i % 20)) and (i > 0) and if_plot:
            plotter.update_plot(pos_xs, pos_ys, pos_hs)

        # Helper Prints
        print(f"Us: {u}")
        print(f"Ws: {w}")
        print(f"x Additions: {final_xs}")
        print(f"y Additions: {final_ys}")
        print(f"New xs: {pos_xs}")
        print(f"New ys: {pos_ys}")
        print(f"Final Headings: {pos_hs}")
        print("---------------------------")

        if i == 1:
            time.sleep(5)


if __name__ == "__main__":
    n_agents = 4
    pos_xs = np.array([-3, -2, -2, -2.1])
    pos_ys = np.array([-2.5, -2, -3, -2.4])
    pos_hs = np.array([0.5, 0.5, 0.5, 0.5])
    initial_positions = np.column_stack((pos_xs, pos_ys))
    initial_headings = pos_hs
    simulate_swarm(n_agents, initial_positions, initial_headings)