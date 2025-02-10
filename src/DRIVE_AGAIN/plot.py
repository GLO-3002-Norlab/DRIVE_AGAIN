from matplotlib.axes import Axes
import matplotlib
import matplotlib.animation
import matplotlib.patches
import matplotlib
import numpy as np
from DRIVE_AGAIN.geofencing import Geofence
from DRIVE_AGAIN.common import Pose


def draw_robot_visualization_figure(ax: Axes, pose: Pose, geofence: Geofence, wheel_base: float) -> None:
    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    draw_geofence(ax, geofence)
    draw_robot(ax, pose, geofence, wheel_base)


def draw_input_space(ax: Axes, commands: np.ndarray) -> None:
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(0, 0.2)

    ax.set_xlabel("v_yaw")
    ax.set_ylabel("v_x")
    ax.set_title("Input space")
    ax.scatter(commands[:, 1], commands[:, 0])


def draw_robot(ax: Axes, pose: Pose, geofence: Geofence, wheel_base: float) -> None:
    x, y, yaw = pose

    robot_color = "green" if geofence.is_point_inside((x, y)) else "red"
    circle = matplotlib.patches.Circle((x, y), wheel_base / 2, color=robot_color)

    wheel_width = 0.1
    wheel_height = wheel_base * 0.8

    left_wheel_x = x - wheel_base / 2 - wheel_width
    left_wheel_y = y - wheel_height / 2

    right_wheel_x = x + wheel_base / 2
    right_wheel_y = y - wheel_height / 2

    wheel_rotation_deg = np.rad2deg(yaw) + 90
    left_wheel = matplotlib.patches.Rectangle(
        (left_wheel_x, left_wheel_y),
        wheel_width,
        wheel_height,
        rotation_point=(x, y),
        angle=wheel_rotation_deg,
        color="black",
    )
    right_wheel = matplotlib.patches.Rectangle(
        (right_wheel_x, right_wheel_y),
        wheel_width,
        wheel_height,
        rotation_point=(x, y),
        angle=wheel_rotation_deg,
        color="black",
    )

    ax.add_patch(circle)
    ax.add_patch(left_wheel)
    ax.add_patch(right_wheel)


def draw_geofence(ax: Axes, geofence: Geofence) -> None:
    geofence_coords = np.array(geofence.polygon.exterior.coords)
    polygon = matplotlib.patches.Polygon(
        geofence_coords, closed=True, edgecolor="blue", facecolor="none", lw=1, linestyle=":"
    )

    ax.add_patch(polygon)
