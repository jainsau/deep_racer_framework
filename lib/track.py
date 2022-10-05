from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from lib.plot import print_borders


def load_waypoints():
    path = str(Path.cwd().joinpath("lib", "rogue_raceway.npy"))
    waypoints = np.load(path)
    return waypoints


def x_perc_width(waypoint, perc_width):
    center_x, center_y, inner_x, inner_y, outer_x, outer_y = waypoint

    delta_x = outer_x - inner_x
    delta_y = outer_y - inner_y

    inner_x_new = inner_x + delta_x / 2 * (1 - perc_width)
    outer_x_new = outer_x - delta_x / 2 * (1 - perc_width)
    inner_y_new = inner_y + delta_y / 2 * (1 - perc_width)
    outer_y_new = outer_y - delta_y / 2 * (1 - perc_width)

    return [center_x, center_y, inner_x_new, inner_y_new, outer_x_new, outer_y_new]


def get_reduced_width_waypoints(perc_width):
    waypoints = load_waypoints()
    new = [x_perc_width(waypoint, perc_width=perc_width) for waypoint in waypoints]
    new = np.asarray(new)
    return new


def get_lines(perc_width=1):
    waypoints = get_reduced_width_waypoints(perc_width)
    center_line = waypoints[:, 0:2]
    inner_border = waypoints[:, 2:4]
    outer_border = waypoints[:, 4:6]
    return center_line, inner_border, outer_border


def plot(center_line, inner_border, outer_border):
    fig = plt.figure(1, figsize=(16, 10))
    ax = fig.add_subplot(111)
    plt.axis("equal")
    print_borders(ax, center_line, inner_border, outer_border)
