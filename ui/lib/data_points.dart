import 'dart:math';

import 'package:flutter/material.dart';
import 'package:ui/drive_socket_message.dart';

class PointsData with ChangeNotifier {
  List<Position> _boundaries = [];
  final List<Position> _points = [];

  void addPoint(Position p) {
    _points.add(p);
    notifyListeners();
  }

  void setBoundaries(List<Position> boundaries) {
    _boundaries = boundaries;
    notifyListeners();
  }
}

class DataPointsWidget extends StatefulWidget {
  final PointsData data;

  const DataPointsWidget({super.key, required this.data});

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
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Center(
              child: Text(
                "Data Points",
                style: Theme.of(context).textTheme.titleLarge,
              ),
            ),
            Expanded(
              child: CustomPaint(
                painter: _DataPointsPainter(
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

class _DataPointsPainter extends CustomPainter {
  final PointsData data;

  const _DataPointsPainter(this.data) : super(repaint: data);

  List<Position> _transformPoints(
      List<Position> points, List<Position> boundaries, Size size) {
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
            Position((p.x - minX) * scaleX + 10, (p.y - minY) * scaleY + 10))
        .toList();
  }

  @override
  void paint(Canvas canvas, Size size) {
    if (data._boundaries.isNotEmpty) {
      final transformedBoundaries =
          _transformPoints(data._boundaries, data._boundaries, size);
      var previous = transformedBoundaries.last;
      for (var b in transformedBoundaries) {
        canvas.drawLine(Offset(b.x.toDouble(), b.y.toDouble()),
            Offset(previous.x.toDouble(), previous.y.toDouble()), Paint());
        previous = b;
      }
    }

    if (data._points.isNotEmpty) {
      final transformedPoints =
          _transformPoints(data._points, data._boundaries, size);
      for (var p in transformedPoints) {
        canvas.drawCircle(Offset(p.x.toDouble(), p.y.toDouble()), 4, Paint());
      }
    }
  }

  @override
  bool shouldRepaint(_DataPointsPainter oldDelegate) => false;

  @override
  bool shouldRebuildSemantics(_DataPointsPainter oldDelegate) => false;
}
