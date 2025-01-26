import 'dart:math';

import 'package:flutter/material.dart';

class DataPointsWidget extends StatefulWidget {
  final List<Point> boundaries;
  final List<Point> points = [];

  DataPointsWidget({super.key, required this.boundaries});

  @override
  State<DataPointsWidget> createState() => _DataPointsWidgetState();
}

class _DataPointsWidgetState extends State<DataPointsWidget>
    with SingleTickerProviderStateMixin {
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
        child: CustomPaint(
          painter: DataPointsPainter(
              boundaries: widget.boundaries, points: widget.points),
        ),
      ),
    );
  }
}

class DataPointsPainter extends CustomPainter {
  final List<Point> boundaries;
  final List<Point> points;

  DataPointsPainter({required this.boundaries, required this.points});

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
    final transformedBoundaries =
        _transformPoints(boundaries, boundaries, size);
    final transformedPoints = _transformPoints(points, boundaries, size);
    var previous = transformedBoundaries.last;
    for (var b in transformedBoundaries) {
      canvas.drawLine(Offset(b.x.toDouble(), b.y.toDouble()),
          Offset(previous.x.toDouble(), previous.y.toDouble()), Paint());
      previous = b;
    }

    for (var p in transformedPoints) {
      canvas.drawCircle(Offset(p.x.toDouble(), p.y.toDouble()), 4, Paint());
    }
  }

  @override
  bool shouldRepaint(DataPointsPainter oldDelegate) => false;
  @override
  bool shouldRebuildSemantics(DataPointsPainter oldDelegate) => false;
}
