import io
import base64
import numpy as np
import matplotlib.pyplot as plt
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading


class Server:
    def __init__(self, start_drive_cb, start_geofencing_cb, stop_geofencing_cb):
        self.app, self.socketio = self.create_server(start_drive_cb, start_geofencing_cb, stop_geofencing_cb)

    def run(self):
        self.socketio.run(self.app, debug=True)

    def update_robot_viz(self, robot_viz_image_b64):
        self.socketio.emit("robot_vizualisation_update", {"image_data": robot_viz_image_b64})

    def update_input_space(self, input_space_image_b64):
        self.socketio.emit("input_space_update", {"image_data": input_space_image_b64})

    def create_server(self, start_drive_cb, start_geofencing_cb, stop_geofencing_cb):
        app = Flask(__name__)
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

        @socketio.on("stop_geofencing")
        def stop_geofencing():
            stop_geofencing_cb()

        return app, socketio
