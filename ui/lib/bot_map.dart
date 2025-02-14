import 'dart:math';

import 'package:flutter/material.dart';

class BotMapData with ChangeNotifier {
  late final List<Point> _fence;
  Point? _position;

  BotMapData({required List<Point> fence}) {
    _fence = fence;
  }

  void setPosition(Point p) {
    _position = p;
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
    final transformedFence = _transformPoints(data._fence, data._fence, size);
    var previous = transformedFence.last;
    for (var b in transformedFence) {
      canvas.drawLine(Offset(b.x.toDouble(), b.y.toDouble()),
          Offset(previous.x.toDouble(), previous.y.toDouble()), Paint());
      previous = b;
    }

    if (data._position != null) {
      final transformedPosition =
          _transformPoints([data._position!], data._fence, size)[0];
      canvas.drawCircle(
          Offset(transformedPosition.x.toDouble(),
              transformedPosition.y.toDouble()),
          4,
          Paint());
    }
  }

  @override
  bool shouldRepaint(BotPositionPainter oldDelegate) => false;
  @override
  bool shouldRebuildSemantics(BotPositionPainter oldDelegate) => false;
}
