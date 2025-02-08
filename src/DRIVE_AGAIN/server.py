import io
import base64
import numpy as np
import matplotlib.pyplot as plt
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading


class Server:
    def __init__(self):
        self.app, self.socketio = self.create_server()

    def run(self):
        self.socketio.run(self.app, debug=True)

    def update_robot_viz(self, robot_viz_image_b64):
        print("update_robot_viz:")
        self.socketio.emit("robot_vizualisation_update", {"image_data": robot_viz_image_b64})

    def update_input_space(self, input_space_image_b64):
        print("update_input_space:")
        self.socketio.emit("update_input_space", {"image_data": input_space_image_b64})

    def create_server(self):
        app = Flask(__name__)
        socketio = SocketIO(app)

        @app.route("/")
        def index():
            return render_template("index.html")

        @socketio.on("start_drive")
        def start_drive():
            print("Starting drive")

        @socketio.on("start_geofencing")
        def start_geofencing():
            print("Starting geofence")

        @socketio.on("stop_geofencing")
        def stop_geofencing():
            print("Stop geofence")

        return app, socketio
