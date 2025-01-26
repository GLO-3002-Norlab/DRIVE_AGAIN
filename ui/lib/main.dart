import 'dart:async';
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
  Mode _mode = Mode.geofencing;

  DataPointsWidget dataPoints = DataPointsWidget(boundaries: const [
    Point(0, 0),
    Point(10, 0),
    Point(10, 10),
    Point(0, 10),
  ]);
  BotMap botMap = BotMap(fence: const [
    Point(0, 0),
    Point(10, 0),
    Point(10, 10),
    Point(0, 10),
  ]);

  void startPressed() {
    setState(() {
      _running = true;
      switch (_mode) {
        case Mode.geofencing:
          startGeofencing();
          break;
        case Mode.drive:
          startDrive();
          break;
      }
    });
  }

  void startGeofencing() {
    var random = Random();
    for (int i = 0; i < 10; i++) {
      dataPoints.points.add(Point(random.nextInt(10), random.nextInt(10)));
    }
  }

  void startDrive() {
    print("start drive");
    var random = Random();
    botMap.position = Point(random.nextInt(10), random.nextInt(10));
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
          dataPoints,
          botMap,
        ],
      ),
    );
  }
}
