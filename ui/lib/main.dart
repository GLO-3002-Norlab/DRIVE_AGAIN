import 'dart:math';

import 'package:flutter/material.dart';
import 'package:ui/bot_map.dart';
import 'package:ui/data_points.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(),
      home: const Home(),
    );
  }
}

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

enum Mode { geofencing, drive }

class _HomeState extends State<Home> {
  bool _running = false;
  Mode _mode = Mode.drive; // TODO - set to geofencing

  final DataPointsWidget dataPointsWidget = DataPointsWidget(
      data: PointsData(boundaries: const [
    Point(0, 0),
    Point(100, 0),
    Point(100, 100),
    Point(0, 100),
  ]));
  final BotMap botMap = BotMap(
      data: BotMapData(fence: const [
    Point(0, 0),
    Point(100, 0),
    Point(100, 100),
    Point(0, 100),
  ]));

  void startPressed() {
    setState(() {
      _running = true;
    });
    switch (_mode) {
      case Mode.geofencing:
        startGeofencing();
        break;
      case Mode.drive:
        startDrive();
        break;
    }
  }

  void startGeofencing() async {
    // TODO
  }

  void startDrive() async {
    var random = Random();
    while (_running) {
      botMap.data.setPosition(Point(random.nextInt(100), random.nextInt(100)));
      for (int i = 0; i < 10; i++) {
        dataPointsWidget.data
            .addPoint(Point(random.nextInt(100), random.nextInt(100)));
      }
      await Future.delayed(const Duration(seconds: 1));
    }
  }

  void stopPressed() {
    setState(() {
      _running = false;
    });
  }

  void onModeChanged(Mode? value) {
    setState(() {
      _mode = value!;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        title: Row(
          children: [
            DropdownButton<Mode>(
              items: [
                DropdownMenuItem(
                  value: Mode.geofencing,
                  enabled: !_running,
                  child: const Text("Geofencing"),
                ),
                DropdownMenuItem(
                  value: Mode.drive,
                  enabled: !_running,
                  child: const Text("Drive"),
                ),
              ],
              onChanged: onModeChanged,
              value: _mode,
            ),
            Container(
              child: _running
                  ? IconButton(
                      onPressed: stopPressed,
                      icon: const Icon(Icons.stop),
                    )
                  : IconButton(
                      onPressed: startPressed,
                      icon: const Icon(Icons.play_arrow),
                    ),
            )
          ],
        ),
      ),
      body: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          dataPointsWidget,
          botMap,
        ],
      ),
    );
  }
}
