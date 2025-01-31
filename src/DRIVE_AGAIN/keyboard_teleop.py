from pynput import keyboard
from pynput.keyboard import Key, KeyCode
import numpy as np

from DRIVE_AGAIN.common import Command


class KeyboardTeleop:
    w_pressed = False
    a_pressed = False
    s_pressed = False
    d_pressed = False

    command: Command = np.array([0.0, -1.0])

    def __init__(self):
        def key_press_lambda(key: Key | KeyCode | None):
            self.on_key_press(key)

        def key_release_lambda(key: Key | KeyCode | None):
            self.on_key_release(key)

        self.keyboard_listener = keyboard.Listener(on_press=key_press_lambda, on_release=key_release_lambda)
        self.keyboard_listener.start()

    def __del__(self):
        self.keyboard_listener.join()

    def get_command(self) -> Command:
        a_x = 0.0
        a_yaw = 0.0
        if self.w_pressed and not self.s_pressed:
            a_x = 1.0
        if self.s_pressed and not self.w_pressed:
            a_x = -1.0
        if self.a_pressed and not self.d_pressed:
            a_yaw = np.pi / 4
        if self.d_pressed and not self.a_pressed:
            a_yaw = -np.pi / 4

        return self.command

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
