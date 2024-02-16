import numpy as np
import time
import plot_swarm_old as plot_swarm
# import debug_print

def wrap_to_pi(x):
    x = x % (3.1415926 * 2)
    x = (x + (3.1415926 * 2)) % (3.1415926 * 2)

    x[x > 3.1415926] = x[x > 3.1415926] - (3.1415926 * 2)
    return x

if_plot = True
if_debug = False
seed_rndm = 12345
rng = np.random.default_rng(seed_rndm)

n_points_x = 5
n_points_y = 5
spacing = 0.8
init_x = 5.0
init_y = 5.0

x_min, x_max = init_x, init_x + n_points_x * spacing - spacing
y_min, y_max = init_y, init_y + n_points_y * spacing - spacing

x_values = np.linspace(x_min, x_max, n_points_x)
y_values = np.linspace(y_min, y_max, n_points_y)
xx, yy = np.meshgrid(x_values, y_values)

pos_xs = xx.ravel() + (rng.random(n_points_x ** 2) * spacing * 0.5) - spacing * 0.25
pos_ys = yy.ravel() + (rng.random(n_points_y ** 2) * spacing * 0.5) - spacing * 0.25  #

pos_hs = (rng.random(n_points_x * n_points_x) * 3.1415926 * 2) - 3.1415926

del n_points_x, n_points_y, spacing, init_x, init_y, x_min, x_max, y_min, y_max, x_values, y_values, xx, yy

# They do circular motion
n_agent = pos_xs.shape[0]
old_centroid_x = np.sum(pos_xs) / n_agent
old_centroid_y = np.sum(pos_ys) / n_agent
dt = 0.05
epsilon = 12.0
sigma_const = 0.7  # 0.4161
sigmas = np.full(n_agent, sigma_const)
umax_const = 0.1
wmax = 1.5708
alpha = 2.0
beta = 0.5
k1 = 0.6
k2 = 0.05
sensing_range = 2.0
run_wall_time = 1000
h_alignment = True
tot_disps = [0.0]
ords = [0.0]
nogs = [0.0]

if if_plot:
    plotter = plot_swarm.SwarmPlotter(n_agent)
# if if_debug:
#     d_plotter = debug_print.DebugPlotter()

t1 = round(time.time() * 1000)
for i in range(int(run_wall_time / dt)):
    xx1, xx2 = np.meshgrid(pos_xs, pos_xs)
    yy1, yy2 = np.meshgrid(pos_ys, pos_ys)
    d_ij_x = xx1 - xx2
    d_ij_y = yy1 - yy2
    d_ij = np.sqrt(np.multiply(d_ij_x, d_ij_x) + np.multiply(d_ij_y, d_ij_y))

    d_ij[d_ij > sensing_range] = np.inf
    d_ij[d_ij == 0.0] = np.inf
    ij_ang = np.arctan2(d_ij_y, d_ij_x) - pos_hs[:, np.newaxis]
    forces = -epsilon * (2 * (sigmas[:, np.newaxis] ** 4 / d_ij ** 5) - (sigmas[:, np.newaxis] ** 2 / d_ij ** 3))
    forces[d_ij == np.inf] = 0.0

    alignment_coss = np.sum(np.cos(pos_hs))
    alignment_sins = np.sum(np.sin(pos_hs))
    alignment_angs = np.arctan2(alignment_sins, alignment_coss)
    alignment_mags = np.sqrt(alignment_coss**2 + alignment_sins**2)
    forces_alignment_x = alignment_mags * np.cos(alignment_angs - pos_hs)
    forces_alignment_y = alignment_mags * np.sin(alignment_angs - pos_hs)

    f_x = alpha * np.sum(np.multiply(forces, np.cos(ij_ang)), axis=1) + \
          int(h_alignment) * beta * forces_alignment_x

    f_y = alpha * np.sum(np.multiply(forces, np.sin(ij_ang)), axis=1) + \
          int(h_alignment) * beta * forces_alignment_y

    u = k1 * f_x + 0.05
    u[u > umax_const] = umax_const
    u[u < 0] = 0.0

    w = k2 * f_y
    w[w > wmax] = wmax
    w[w < -wmax] = -wmax

    pos_xs = pos_xs + np.multiply(u, np.cos(pos_hs)) * dt
    pos_ys = pos_ys + np.multiply(u, np.sin(pos_hs)) * dt
    pos_hs = wrap_to_pi(pos_hs + w * dt)

    if (not (i % 100)) and (i > 10) and (if_plot or if_debug):
        plotter.update_plot(pos_xs, pos_ys, pos_hs)
        # if if_debug:
        #     d_plotter.plot_data(np.linspace(0, i, len(ords)), ords, tot_disps)
        if if_plot:
            plotter.update_plot(pos_xs, pos_ys, pos_hs)