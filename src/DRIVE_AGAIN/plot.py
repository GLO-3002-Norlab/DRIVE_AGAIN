from matplotlib.axes import Axes
import matplotlib
import matplotlib.animation
import matplotlib.patches
import matplotlib
import numpy as np
from DRIVE_AGAIN.geofencing import Geofence
from DRIVE_AGAIN.common import Pose


def draw_robot_visualization_figure(ax: Axes, pose: Pose, geofence_points: np.ndarray, wheel_base: float) -> None:
    ax.clear()
    ax.set_xlim(-15, 15)
    ax.set_ylim(-15, 15)
    draw_incomplete_geofence(ax, geofence_points)
    draw_robot(ax, pose, wheel_base)


def draw_input_space(ax: Axes, commands: np.ndarray) -> None:
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(0, 0.2)

    ax.set_xlabel("v_yaw")
    ax.set_ylabel("v_x")
    ax.set_title("Input space")

    if len(commands) > 0:
        ax.scatter(commands[:, 1], commands[:, 0])


def draw_robot(ax: Axes, pose: Pose, wheel_base: float) -> None:
    x, y, _, _, _, yaw = pose

    circle = matplotlib.patches.Circle((x, y), wheel_base / 2, color="green")

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


def draw_incomplete_geofence(ax: Axes, geofence_points: np.ndarray) -> None:
    if len(geofence_points) > 0:
        ax.plot(geofence_points[:, 0], geofence_points[:, 1], marker="o", linestyle="-", color="r", markersize=4)


def draw_geofence(ax: Axes, geofence: Geofence) -> None:
    geofence_coords = np.array(geofence.polygon.exterior.coords)
    polygon = matplotlib.patches.Polygon(
        geofence_coords, closed=True, edgecolor="blue", facecolor="none", lw=1, linestyle=":", markersize=4
    )

    ax.add_patch(polygon)
