import 'dart:math';

import 'package:flutter/material.dart';
import 'package:ui/drive_socket_message.dart';

class BotMapData with ChangeNotifier {
  List<Position> _fence = [];
  List<Position>? _mapBounds;
  Pose? _position;

  void setPosition(Pose p) {
    _position = p;
    notifyListeners();
  }

  void setFence(List<Position> fence) {
    _fence = fence;
    _mapBounds ??= fence;
    notifyListeners();
  }

  void setBounds(List<Position> bounds) {
    _mapBounds = bounds;
    notifyListeners();
  }
}

class BotMap extends StatefulWidget {
  final BotMapData data;

  const BotMap({super.key, required this.data});

  @override
  State<BotMap> createState() => _BotMapState();
}

class _BotMapState extends State<BotMap> {
  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    var sizes = MediaQuery.sizeOf(context);
    var size = max(sizes.width / 2, sizes.height / 2);
    return SizedBox(
      width: size,
      height: size,
      child: Container(
        color: Colors.white,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Center(
              child: Text(
                "Bot Map",
                style: Theme.of(context).textTheme.titleLarge,
              ),
            ),
            Expanded(
              child: CustomPaint(
                painter: BotPositionPainter(
                  widget.data,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class BotPositionPainter extends CustomPainter {
  final BotMapData data;

  BotPositionPainter(this.data) : super(repaint: data);

  List<Position> _transformPoints(
      List<Position> points, List<Position> boundaries, Size size) {
    final x = boundaries.map((p) => p.x);
    final y = boundaries.map((p) => p.y);
    final minX = x.reduce(min);
    final maxX = x.reduce(max);
    final minY = y.reduce(min);
    final maxY = y.reduce(max);

    final scaleX = (size.width - 20) / (maxX - minX);
    final scaleY = (size.height - 20) / (maxY - minY);
    return points
        .map((p) =>
            Position((p.x - minX) * scaleX + 10, (p.y - minY) * scaleY + 10))
        .toList();
  }

  @override
  void paint(Canvas canvas, Size size) {
    if (data._fence.isNotEmpty) {
      final transformedFence =
          _transformPoints(data._fence, data._mapBounds ?? data._fence, size);
      var previous = transformedFence.last;
      for (var b in transformedFence) {
        canvas.drawLine(Offset(b.x.toDouble(), b.y.toDouble()),
            Offset(previous.x.toDouble(), previous.y.toDouble()), Paint());
        previous = b;
      }
    }

    if (data._position != null) {
      final transformedPosition =
          _transformPoints([data._position!], data._fence, size)[0];
      canvas.drawCircle(
        Offset(transformedPosition.x, transformedPosition.y),
        8,
        Paint(),
      );
      if (data._position!.yaw != null) {
        canvas.drawCircle(
          Offset(
            transformedPosition.x + (16 * cos(data._position!.yaw!)),
            transformedPosition.y + (16 * sin(data._position!.yaw!)),
          ),
          6,
          Paint(),
        );
      }
    }
  }

  @override
  bool shouldRepaint(BotPositionPainter oldDelegate) => false;
  @override
  bool shouldRebuildSemantics(BotPositionPainter oldDelegate) => false;
}
