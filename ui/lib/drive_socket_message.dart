import 'dart:math';

enum MessageType {
  startGeofencing,
  startDrive,
  dataBounds,
  geoFence,
  botPosition,
  data
}

class DriveSocketMessage {
  MessageType type;
  List<Point>? points;
  Point? point;

  DriveSocketMessage(this.type, {this.points, this.point});

  static Point pointFromJson(Map<String, dynamic> json) {
    return Point(json["x"], json["y"]);
  }

  static Map<String, dynamic> pointToJson(Point p) {
    return {"x": p.x, "y": p.y};
  }

  DriveSocketMessage.fromJson(Map<String, dynamic> json)
      : type = MessageType.values.firstWhere((e) => e.name == json["type"]),
        points = [for (var p in json["points"]) pointFromJson(p)],
        point = pointFromJson(json["point"]);

  Map<String, dynamic> toJson() {
    return {
      'type': type.name,
      'points': points == null ? [] : [for (var p in points!) pointToJson(p)],
      'point': points == null ? null : pointToJson(point!),
    };
  }
}
