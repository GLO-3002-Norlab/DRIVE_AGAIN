try:
    from pynput import keyboard
except ImportError:
    keyboard = None
    Key = None
    KeyCode = None

import numpy as np

from DRIVE_AGAIN.common import Command

V_X_MAX = 5.0
V_YAW_MAX = np.pi


class KeyboardListener:
    def __init__(self):
        if keyboard is None:
            print("Running in headless environment, keyboard listener is disabled")
            return

        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            self.pressed_keys.add(key.char)
        except AttributeError:
            self.pressed_keys.add(key)

    def on_release(self, key):
        try:
            self.pressed_keys.discard(key.char)
        except AttributeError:
            self.pressed_keys.discard(key)

    def get_pressed_keys(self):
        return list(self.pressed_keys)


class KeyboardTeleop:
    FORWARD_KEY = "w"
    BACKWARD_KEY = "s"
    LEFT_KEY = "a"
    RIGHT_KEY = "d"
    GEOFENCE_KEY = "g"
    DEADMAN_KEY = "x"

    def __init__(self):
        self.keyboard_listener = KeyboardListener()
        self.v_x = 0.0
        self.v_yaw = 0.0
        self.last_timestamp_ns: None | float = None

    def is_deadman_key_pressed(self):
        return self.DEADMAN_KEY in self.keyboard_listener.get_pressed_keys()

    def is_geofence_key_pressed(self):
        return self.GEOFENCE_KEY in self.keyboard_listener.get_pressed_keys()

    def is_key_pressed(self, key) -> bool:
        return key in self.keyboard_listener.get_pressed_keys()

    def get_robot_command(self, timestamp_ns: float) -> Command:
        if self.last_timestamp_ns is None:
            self.last_timestamp_ns = timestamp_ns
            return np.array([0.0, 0.0])

        dt_s = (timestamp_ns - self.last_timestamp_ns) * 1e-9
        self.last_timestamp_ns = timestamp_ns

        forward_press = self.is_key_pressed(self.FORWARD_KEY)
        backward_press = self.is_key_pressed(self.BACKWARD_KEY)
        left_press = self.is_key_pressed(self.LEFT_KEY)
        right_press = self.is_key_pressed(self.RIGHT_KEY)

        a_x = 0.0
        a_yaw = 0.0
        if forward_press and not backward_press:
            a_x = 2.0
        if backward_press and not forward_press:
            a_x = -2.0
        if left_press and not right_press:
            a_yaw = np.pi / 2.0
        if right_press and not left_press:
            a_yaw = -np.pi / 2.0

        self.v_x = self.v_x + a_x * dt_s
        self.v_yaw = self.v_yaw + a_yaw * dt_s

        self.v_x *= 0.5
        self.v_yaw *= 0.5

        self.v_x = np.clip(self.v_x, -V_X_MAX, V_X_MAX)
        self.v_yaw = np.clip(self.v_yaw, -V_YAW_MAX, V_YAW_MAX)

        return np.array([self.v_x, self.v_yaw])
