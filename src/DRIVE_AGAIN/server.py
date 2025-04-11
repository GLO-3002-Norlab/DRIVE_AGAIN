import base64
import io

import matplotlib.pyplot as plt
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO
from matplotlib.figure import Figure

from DRIVE_AGAIN.common import Pose
from DRIVE_AGAIN.plot import draw_input_space, draw_robot_visualization_figure


class Server:
    def __init__(
        self, start_drive_cb, start_geofencing_cb, save_dataset_cb, load_geofence_cb, get_datasets_cb, skip_command_cb
    ):
        self.app, self.socketio = self.create_server(
            start_drive_cb, start_geofencing_cb, save_dataset_cb, load_geofence_cb, get_datasets_cb, skip_command_cb
        )

        self.fig_viz, self.ax_viz = plt.subplots()
        self.fig_input_space, self.ax_input_space = plt.subplots()

    def run(self):
        self.socketio.run(self.app, host="0.0.0.0", debug=False, allow_unsafe_werkzeug=True, use_reloader=False)

    def encode_fig_to_b64(self, fig: Figure):
        buffer = io.BytesIO()
        fig.savefig(buffer, format="png")
        return base64.b64encode(buffer.getvalue()).decode("utf-8")

    def update_robot_viz(self, robot_pose: Pose, geofence_points: np.ndarray, wheel_base: float):
        draw_robot_visualization_figure(self.ax_viz, robot_pose, geofence_points, wheel_base)
        viz_b64 = self.encode_fig_to_b64(self.fig_viz)

        self.socketio.emit("robot_vizualisation_update", {"image_data": viz_b64})

    def update_input_space(self, commands: np.ndarray):
        draw_input_space(self.ax_input_space, commands)
        input_space_b64 = self.encode_fig_to_b64(self.fig_input_space)

        self.socketio.emit("input_space_update", {"image_data": input_space_b64})

    def skippable_state_start(self):
        self.socketio.emit("skippable_state_start")

    def skippable_state_end(self):
        self.socketio.emit("skippable_state_end")

    def create_server(
        self, start_drive_cb, start_geofencing_cb, save_dataset_cb, load_geofence_cb, get_datasets_cb, skip_command_cb
    ):
        app = Flask(__name__, static_url_path="/static", static_folder="web/static", template_folder="web/templates")
        socketio = SocketIO(app)

        @app.route("/")
        def index():
            return render_template("index.html")

        @socketio.on("start_drive")
        def start_drive():
            start_drive_cb()

        @socketio.on("start_geofencing")
        def start_geofencing():
            start_geofencing_cb()

        @socketio.on("save_dataset")
        def save_dataset(data):
            dataset_name = data.get("name")
            save_dataset_cb(dataset_name)

        @socketio.on("update_datasets")
        def update_datasets():
            self.socketio.emit("datasets", get_datasets_cb())

        @socketio.on("load_geofence")
        def load_geofence(data):
            dataset_name = data.get("name")
            load_geofence_cb(dataset_name)

        @socketio.on("skip_command")
        def skip_command():
            skip_command_cb()

        return app, socketio
