import shapely.geometry.polygon as polygon


def plot_coordinates(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, ".", color="black", zorder=1)


def plot_line(ax, ob):
    x, y = ob.xy
    ax.plot(
        x, y, color="gray", alpha=0.7, linewidth=3, solid_capstyle="round", zorder=2
    )


def print_borders(ax, waypoints, inner_border_waypoints, outer_border_waypoints):
    line = polygon.LineString(waypoints)
    plot_coordinates(ax, line)
    plot_line(ax, line)

    line = polygon.LineString(inner_border_waypoints)
    plot_coordinates(ax, line)
    plot_line(ax, line)

    line = polygon.LineString(outer_border_waypoints)
    plot_coordinates(ax, line)
    plot_line(ax, line)
