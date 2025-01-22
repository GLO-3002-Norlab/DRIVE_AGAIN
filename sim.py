import matplotlib
import matplotlib.animation
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np

WHEEL_BASE = 0.5

Pose = np.ndarray # x, y, yaw
Command = np.ndarray # v_x, omega_z


def draw_robot(ax: Axes, pose: Pose) -> None:
    x, y, yaw = pose

    ax.clear()
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)

    # Draw circle for robot
    circle = plt.Circle((x, y), WHEEL_BASE / 2, color='r') # type: ignore

    # Draw wheels
    wheel_width = 0.1
    wheel_height = WHEEL_BASE * 0.8

    left_wheel_x = x - WHEEL_BASE / 2 - wheel_width
    left_wheel_y = y - wheel_height / 2

    right_wheel_x = x + WHEEL_BASE / 2
    right_wheel_y = y - wheel_height / 2

    wheel_rotation_deg = np.rad2deg(yaw)+90
    left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y), wheel_width, wheel_height, rotation_point=(x, y), angle=wheel_rotation_deg, color='black') # type: ignore
    right_wheel = plt.Rectangle((right_wheel_x, right_wheel_y), wheel_width, wheel_height, rotation_point=(x, y), angle=wheel_rotation_deg, color='black') # type: ignore
    
    ax.add_artist(circle)
    ax.add_artist(left_wheel)
    ax.add_artist(right_wheel)


def apply_command(u: Command):
    x, y, yaw = pose
    v_x, omega_z = u

    x += v_x * np.cos(yaw)
    y += v_x * np.sin(yaw)
    yaw += omega_z

    return np.array([x, y, yaw])


if __name__ == '__main__':
    pose = np.array([1, 1, np.pi/2]) 
    command = np.array([0.1, 0.1])

    fig, ax = plt.subplots()

    def update(frame):
        global pose
        pose = apply_command(command)
        draw_robot(ax, pose)

    ani = matplotlib.animation.FuncAnimation(fig, update, frames=200, interval=50) # type: ignore
    plt.show()