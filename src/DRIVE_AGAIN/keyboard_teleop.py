from pynput import keyboard
from pynput.keyboard import Key, KeyCode
import numpy as np

from DRIVE_AGAIN.common import Command

V_X_MAX = 5.0
V_YAW_MAX = np.pi


class KeyboardTeleop:
    w_pressed = False  # Forward
    a_pressed = False  # Left
    s_pressed = False  # Backward
    d_pressed = False  # Right

    x_pressed = False  # Deadman switch

    def __init__(self):
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()
        self.v_x = 0.0
        self.v_yaw = 0.0
        self.last_timestamp_ns: None | float = None

    def is_deadman_switch_pressed(self):
        return self.x_pressed

    def is_teleop_active(self):
        return self.w_pressed or self.a_pressed or self.s_pressed or self.d_pressed

    def __del__(self):
        self.keyboard_listener.join()

    def get_command(self, timestamp_ns: float) -> Command:
        if self.last_timestamp_ns is None:
            self.last_timestamp_ns = timestamp_ns
            return np.array([0.0, 0.0])

        dt_s = (timestamp_ns - self.last_timestamp_ns) * 1e-9
        self.last_timestamp_ns = timestamp_ns

        a_x = 0.0
        a_yaw = 0.0
        if self.w_pressed and not self.s_pressed:
            a_x = 2.0
        if self.s_pressed and not self.w_pressed:
            a_x = -2.0
        if self.a_pressed and not self.d_pressed:
            a_yaw = np.pi / 2.0
        if self.d_pressed and not self.a_pressed:
            a_yaw = -np.pi / 2.0

        self.v_x = self.v_x + a_x * dt_s
        self.v_yaw = self.v_yaw + a_yaw * dt_s

        self.v_x *= 0.5
        self.v_yaw *= 0.5

        self.v_x = np.clip(self.v_x, -V_X_MAX, V_X_MAX)
        self.v_yaw = np.clip(self.v_yaw, -V_YAW_MAX, V_YAW_MAX)

        return np.array([self.v_x, self.v_yaw])

    def on_key_press(self, key: Key | KeyCode | None):
        if key is None or isinstance(key, Key):
            return

        if key.char == "w":
            self.w_pressed = True
        if key.char == "s":
            self.s_pressed = True
        if key.char == "a":
            self.a_pressed = True
        if key.char == "d":
            self.d_pressed = True
        if key.char == "x":
            self.x_pressed = True

    def on_key_release(self, key: Key | KeyCode | None):
        if key is None or isinstance(key, Key):
            return

        if key.char == "w":
            self.w_pressed = False
        if key.char == "s":
            self.s_pressed = False
        if key.char == "a":
            self.a_pressed = False
        if key.char == "d":
            self.d_pressed = False
        if key.char == "x":
            self.x_pressed = False
