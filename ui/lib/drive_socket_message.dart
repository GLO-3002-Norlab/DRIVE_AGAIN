enum MessageType {
  startGeofencing,
  stopGeofencing,
  startDrive,
  stopDrive,
  dataBounds,
  geoFence,
  botPose,
  data,
  files,
  invalid
}

class Position {
  static const String X_KEY = "x";
  static const String Y_KEY = "y";
  double x;
  double y;

  Position(this.x, this.y);

  Position.fromJson(Map<String, dynamic> json)
      : x = json[X_KEY],
        y = json[Y_KEY];

  Map<String, dynamic> toJson() {
    return {X_KEY: x, Y_KEY: y};
  }
}

class Pose extends Position {
  static const String YAW_KEY = "yaw";
  double? yaw;

  Pose(double x, double y, {this.yaw}) : super(x, y);

  Pose.fromJson(Map<String, dynamic> json)
      : yaw = json.containsKey(YAW_KEY) && json[YAW_KEY] != null
            ? json[YAW_KEY]
            : null,
        super.fromJson(json);

  @override
  Map<String, dynamic> toJson() {
    var value = super.toJson();
    value[YAW_KEY] = yaw;
    return value;
  }

  Position asPosition() {
    return Position(x, y);
  }
}

class DriveSocketMessage {
  static const String TYPE_KEY = "type";
  static const String POSITIONS_KEY = "positions";
  static const String POSE_KEY = "pose";
  MessageType type;
  List<Position>? positions;
  Pose? pose;

  DriveSocketMessage(this.type, {this.positions, this.pose});

  DriveSocketMessage.fromJson(Map<String, dynamic> json)
      : type = MessageType.values.firstWhere((e) => e.name == json[TYPE_KEY],
            orElse: () => MessageType.invalid),
        positions =
            json.containsKey(POSITIONS_KEY) && (json[POSITIONS_KEY] != null)
                ? [for (var p in json[POSITIONS_KEY]) Pose.fromJson(p)]
                : null,
        pose = json.containsKey(POSE_KEY) && json[POSE_KEY] != null
            ? Pose.fromJson(json[POSE_KEY])
            : null;

  Map<String, dynamic> toJson() {
    return {
      TYPE_KEY: type.name,
      POSITIONS_KEY:
          positions != null ? [] : [for (var p in positions!) p.toJson()],
      POSE_KEY: pose != null ? null : pose!.toJson(),
    };
  }
}
