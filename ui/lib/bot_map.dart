import 'dart:math';

import 'package:flutter/material.dart';

class BotMap extends StatefulWidget {
  final List<Point> fence;
  Point? position;

  BotMap({super.key, required this.fence});

  @override
  State<BotMap> createState() => _BotMapState();
}

class _BotMapState extends State<BotMap> {
  @override
  Widget build(BuildContext context) {
    var sizes = MediaQuery.sizeOf(context);
    var size = max(sizes.width / 2, sizes.height / 2);
    return SizedBox(
      width: size,
      height: size,
      child: Container(
        color: Colors.white,
        child: CustomPaint(
          painter: BotPositionPainter(
              fence: widget.fence, position: widget.position),
        ),
      ),
    );
  }
}

class BotPositionPainter extends CustomPainter {
  final List<Point> fence;
  final Point? position;

  BotPositionPainter({required this.fence, required this.position});

  List<Point> _transformPoints(
      List<Point> points, List<Point> boundaries, Size size) {
    final x = boundaries.map((p) => p.x);
    final y = boundaries.map((p) => p.y);
    final minX = x.reduce(min);
    final maxX = x.reduce(max);
    final scaleX = (size.width - 20) / (maxX - minX);
    final minY = y.reduce(min);
    final maxY = y.reduce(max);
    final scaleY = (size.height - 20) / (maxY - minY);
    return points
        .map((p) =>
            Point((p.x - minX) * scaleX + 10, (p.y - minY) * scaleY + 10))
        .toList();
  }

  @override
  void paint(Canvas canvas, Size size) {
    final transformedFence = _transformPoints(fence, fence, size);
    var previous = transformedFence.last;
    for (var b in transformedFence) {
      canvas.drawLine(Offset(b.x.toDouble(), b.y.toDouble()),
          Offset(previous.x.toDouble(), previous.y.toDouble()), Paint());
      previous = b;
    }

    print(position);
    if (position != null) {
      final transformedPosition = _transformPoints([position!], fence, size)[0];
      print(transformedPosition);
      canvas.drawCircle(
          Offset(transformedPosition.x.toDouble(), transformedPosition.y.toDouble()), 4, Paint());
    }
  }

  @override
  bool shouldRepaint(BotPositionPainter oldDelegate) => false;
  @override
  bool shouldRebuildSemantics(BotPositionPainter oldDelegate) => false;
}
