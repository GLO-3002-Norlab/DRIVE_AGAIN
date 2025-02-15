enum MessageType {
  startGeofencing,
  startDrive,
  dataBounds,
  geoFence,
  botPose,
  data,
  files,
  invalid
}

class Pose {
  static const String X_KEY = "x";
  static const String Y_KEY = "y";
  static const String YAW_KEY = "yaw";
  double x;
  double y;
  double? yaw;

  Pose(this.x, this.y, {this.yaw});

  Pose.fromJson(Map<String, dynamic> json)
      : x = json[X_KEY],
        y = json[Y_KEY],
        yaw = json.containsKey(YAW_KEY) && json[YAW_KEY] != null
            ? json[YAW_KEY]
            : null;

  Map<String, dynamic> toJson() {
    return {X_KEY: x, Y_KEY: y, YAW_KEY: yaw};
  }
}

// enum MessageKeys {
//   TYPE_KEY("type"),
//   POSES_KEY("poses"),
//   POSE_KEY("pose");

//   final String value;
//   const MessageKeys(this.value);
// }

class DriveSocketMessage {
  static const String TYPE_KEY = "type";
  static const String POSES_KEY = "poses";
  static const String POSE_KEY = "pose";
  MessageType type;
  List<Pose>? poses;
  Pose? pose;
  // List<String> strings;

  DriveSocketMessage(this.type, {this.poses, this.pose});

  DriveSocketMessage.fromJson(Map<String, dynamic> json)
      : type = MessageType.values.firstWhere((e) => e.name == json[TYPE_KEY],
            orElse: () => MessageType.invalid),
        poses = json.containsKey(POSES_KEY) && (json[POSES_KEY] != null)
            ? [for (var p in json[POSES_KEY]) Pose.fromJson(p)]
            : null,
        pose = json.containsKey(POSE_KEY) && json[POSE_KEY] != null
            ? Pose.fromJson(json[POSE_KEY])
            : null;

  Map<String, dynamic> toJson() {
    return {
      'type': type.name,
      'points': poses != null ? [] : [for (var p in poses!) p.toJson()],
      'point': pose != null ? null : pose!.toJson(),
    };
  }
}
