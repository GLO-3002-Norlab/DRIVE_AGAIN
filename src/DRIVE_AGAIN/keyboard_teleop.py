from pynput import keyboard
from pynput.keyboard import Key, KeyCode
import numpy as np

from DRIVE_AGAIN.common import Command


class KeyboardTeleop:
    w_pressed = False
    a_pressed = False
    s_pressed = False
    d_pressed = False

    def __init__(self):
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()

    def __del__(self):
        self.keyboard_listener.join()

    def get_command(self) -> Command:
        v_x = 0.0
        v_yaw = 0.0
        if self.w_pressed and not self.s_pressed:
            v_x = 0.2
        if self.s_pressed and not self.w_pressed:
            v_x = -0.2
        if self.a_pressed and not self.d_pressed:
            v_yaw = np.pi / 16.0
        if self.d_pressed and not self.a_pressed:
            v_yaw = -np.pi / 16.0

        return np.array([v_x, v_yaw])

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
