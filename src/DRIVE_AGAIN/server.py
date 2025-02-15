import base64
import io
import json
import threading
from typing import Any, Callable

import matplotlib.pyplot as plt
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO

from DRIVE_AGAIN.common import Command, Pose
from DRIVE_AGAIN.geofencing import Geofence


class MessageType:
  startGeofencing="startGeofencing"
  stopGeofencing="stopGeofencing"
  startDrive="startDrive"
  stopDrive="stopDrive"
  dataBounds="dataBounds"
  geoFence="geoFence"
  botPose="botPose"
  data="data"
  files="files"


class DriveSocketMessage:
    type: str
    positions: list[dict[str, float]] | None
    pose: dict[str, float] | None

    def __init__(self, type, positions: list[dict[str, float]] | None = None, pose: dict[str, float] | None = None):
        self.type = type
        self.positions = positions
        self.pose = pose

    def toJson(self) -> str:
        return json.dumps({
            "type": self.type,
            "positions": self.positions,
            "pose": self.pose
        })

class Server:
    connect_cb: Callable[[], None] | None = None

    def __init__(self, start_geofencing_cb, stop_geofencing_cb, start_drive_cb, stop_drive_cb):
        self.app, self.socketio = self.create_server(start_geofencing_cb, stop_geofencing_cb, start_drive_cb, stop_drive_cb)

    def run(self):
        self.socketio.run(self.app, debug=True)

    def update_robot_position(self, pose: Pose):
        x, y, yaw = pose
        msg = DriveSocketMessage(MessageType.botPose, pose={"x": x, "y": y, "yaw": yaw})
        self.socketio.emit("data", msg.toJson())
    
    def send_geofence(self, geofence: Geofence):
        coords: zip[tuple[float, float]] = zip(*geofence.polygon.exterior.coords.xy)
        msg = DriveSocketMessage(MessageType.geoFence, positions=[{"x": p[0], "y": p[1]} for p in coords])
        self.socketio.emit("data", msg.toJson())

    def send_data_point(self, command: Command):
        msg = DriveSocketMessage(MessageType.data, pose={"x": command[1], "y": command[0]})
        self.socketio.emit("data", msg.toJson())

    def send_data_bounds(self, bounds: list[Pose]):
        msg = DriveSocketMessage(MessageType.dataBounds, positions=[{"x": p[0], "y": p[1]} for p in bounds])
        self.socketio.emit("data", msg.toJson())

    def create_server(self, start_geofencing_cb, stop_geofencing_cb, start_drive_cb, stop_drive_cb):
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

        @socketio.on("connect")
        def connection():
            if self.connect_cb is not None:
                self.connect_cb()

        @socketio.on(MessageType.startGeofencing)
        def start_geofencing():
            start_geofencing_cb()

        @socketio.on(MessageType.stopGeofencing)
        def stop_geofencing():
            stop_geofencing_cb()
            
        @socketio.on(MessageType.startDrive)
        def start_drive():
            start_drive_cb()
            
        @socketio.on(MessageType.stopDrive)
        def stop_drive():
            stop_drive_cb()

        return app, socketio
