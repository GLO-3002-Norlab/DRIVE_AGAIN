import 'dart:async';
import 'dart:io';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:ui/bot_map.dart';
import 'package:ui/data_points.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'dart:convert' as json;

import 'drive_socket_message.dart';

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
  final WebSocketChannel channel =
      WebSocketChannel.connect(Uri.parse('ws://localhost:8000'));

  @override
  void initState() {
    super.initState();

    listenOnWebSocket();
  }

  void handleMessage(DriveSocketMessage message) {
    switch (message.type) {
      case MessageType.botPosition:
        botMap.data.setPosition(message.point!);
        break;
      case MessageType.startGeofencing:
        // TODO
        break;
      case MessageType.startDrive:
        // TODO
        break;
      case MessageType.dataBounds:
        // TODO
        break;
      case MessageType.geoFence:
        // TODO
        break;
      case MessageType.botPosition:
        // TODO
        break;
      case MessageType.data:
        // TODO
        break;
    }
  }

  Future<void> listenOnWebSocket() async {
    try {
      await channel.ready;
      print("Channel is connected");

      channel.stream.listen((value) async {
        print("Received event: [$value]");
        DriveSocketMessage message =
            DriveSocketMessage.fromJson(json.jsonDecode(value));
        handleMessage(message);
      }).onDone(() {
        print("Channel disconnected");
      });
    } on SocketException {
      if (kDebugMode) {
        print("Failed to connect to socket");
      }
    }
    return;
  }

  void startPressed() {
    // var message =
    //     DriveSocketMessage(MessageType.startDrive, point: Point(1, 2));
    // channel.sink.add(JSON.jsonEncode(message));
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
