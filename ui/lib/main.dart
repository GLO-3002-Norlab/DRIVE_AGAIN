import 'dart:async';
import 'dart:io';
import 'dart:js_interop';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart';
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
  static const String SOCKET_URL = "http://127.0.0.1:5000";

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
      data: BotMapData(fence: [
    Pose(0, 0),
    Pose(100, 0),
    Pose(100, 100),
    Pose(0, 100),
  ]));
  Socket socket = io(SOCKET_URL, <String, dynamic>{
    'transports': ['websocket'],
    "autoReconnect": false,
  })
    ..onConnect((_) => print("Connection established with: $SOCKET_URL"))
    ..onDisconnect((_) => print("Disconnected from socket"));

  @override
  void initState() {
    super.initState();
    socket.on("data", (data) => handleMessage(data));
  }

  void handleMessage(String data) {
    print("Received data: ");
    print(data);
    DriveSocketMessage message =
        DriveSocketMessage.fromJson(json.jsonDecode(data));
    switch (message.type) {
      case MessageType.botPose:
        botMap.data.setPosition(message.pose!);
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
      case MessageType.data:
        // TODO
        break;
      case MessageType.files:
        // TODO
        break;
      case MessageType.invalid:
        print("Invalid message received");
        break;
    }
  }

  Future<void> listenOnWebSocket() async {
    // channel ??= WebSocketChannel.connect(Uri.parse(SOCKET_URL));
    // try {
    //   await channel!.ready.then((_) {
    //     print("Channel is connected");
    //     channel!.stream.listen((value) async {
    //       print("Received event: [$value]");
    //       DriveSocketMessage message =
    //           DriveSocketMessage.fromJson(json.jsonDecode(value));
    //       handleMessage(message);
    //     }).onDone(() {
    //       print("Channel disconnected");
    //     });
    //   }).onError((error, stackTrace) {
    //     print("Error on connection with socket at: $SOCKET_URL");
    //     print(error);
    //   });
    // } on WebSocketChannelException {
    //   print("error on connect!!");
    // } on SocketException {
    //   // if (kDebugMode) {
    //   print("Failed to connect to socket");
    //   // }
    // }

    // print("1: ${socket.connected}");
  }

  void startPressed() {
    setState(() {
      _running = true;
    });
    listenOnWebSocket();
    // var message =
    //     DriveSocketMessage(MessageType.startDrive, point: Point(1, 2));
    // channel.sink.add(JSON.jsonEncode(message));
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
      botMap.data.setPosition(
          Pose(random.nextDouble() * 100, random.nextDouble() * 100));
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
    print("2: ${socket.connected}");
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
