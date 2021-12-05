import numpy as np
from utils import draw_sphere_marker


def plot_points(points, color=(0, 0, 0, 1)):
    for point in points:
        plot_point(point, color)


def plot_point(point, color=(0, 0, 0, 1)):
    draw_sphere_marker((point[0], point[1], 0.4), 0.05, color)
