import base64
import io
import json
import threading
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO

from DRIVE_AGAIN.common import Pose


class MessageType:
  startGeofencing="startGeofencing"
  startDrive="startDrive"
  dataBounds="dataBounds"
  geoFence="geoFence"
  botPose="botPose"
  data="data"
  files="files"


class DriveSocketMessage:
    type: str
    poses: dict[str, int] | None
    pose: dict[str, int] | None
    # // List<String> strings

    def __init__(self, type, poses: dict[str, int] | None = None, pose: dict[str, int] | None = None):
        self.type = type
        self.poses = poses
        self.pose = pose

    def toJson(self) -> str:
        return json.dumps({
            "type": self.type,
            "poses": self.poses,
            "pose": self.pose
        })

class Server:
    def __init__(self, start_drive_cb, start_geofencing_cb, stop_geofencing_cb):
        self.app, self.socketio = self.create_server(start_drive_cb, start_geofencing_cb, stop_geofencing_cb)

    def run(self):
        self.socketio.run(self.app, debug=True)

    def update_robot_position(self, pose: Pose):
        x, y, yaw = pose
        msg = DriveSocketMessage(MessageType.botPose, pose={"x": x, "y": y})
        print("new pos message:\n" + msg.toJson())
        self.socketio.emit("data", msg.toJson())
        
    def update_robot_viz(self, robot_viz_image_b64):
        self.socketio.emit("robot_vizualisation_update", {"image_data": robot_viz_image_b64})

    def update_input_space(self, input_space_image_b64):
        self.socketio.emit("input_space_update", {"image_data": input_space_image_b64})

    def create_server(self, start_drive_cb, start_geofencing_cb, stop_geofencing_cb):
        app = Flask(__name__)
        socketio = SocketIO(app)
        socketio.init_app(app, cors_allowed_origins="*")

        @app.route("/")
        def index():
            print("call to /")
            return render_template("index.html")

        @socketio.on("data")
        def message(data):
            print("received data:\n" + data)

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
