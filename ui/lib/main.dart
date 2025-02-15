import 'package:flutter/material.dart';
import 'package:socket_io_client/socket_io_client.dart';
import 'package:ui/bot_map.dart';
import 'package:ui/data_points.dart';
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

  final DataPointsWidget dataPointsWidget =
      DataPointsWidget(data: PointsData());
  final BotMap botMap = BotMap(data: BotMapData());
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
      case MessageType.stopGeofencing:
        // TODO
        break;
      case MessageType.startDrive:
        // TODO
        break;
      case MessageType.stopDrive:
        // TODO
        break;
      case MessageType.dataBounds:
        if (message.positions != null) {
          dataPointsWidget.data.setBoundaries(message.positions!);
        }
        break;
      case MessageType.geoFence:
        if (message.positions != null) {
          botMap.data.setFence(message.positions!);
        }
        break;
      case MessageType.data:
        if (message.pose != null) {
          dataPointsWidget.data.addPoint(message.pose!.asPosition());
        }
        break;
      case MessageType.files:
        // TODO
        break;
      case MessageType.invalid:
        print("Invalid message received");
        break;
      default:
        break;
    }
  }

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

  void stopPressed() {
    setState(() {
      _running = false;
    });
    switch (_mode) {
      case Mode.geofencing:
        stopGeofencing();
        break;
      case Mode.drive:
        stopDrive();
        break;
    }
  }

  void startGeofencing() async {
    socket.emit(MessageType.startGeofencing.name);
  }

  void startDrive() async {
    socket.emit(MessageType.startDrive.name);
  }

  void stopGeofencing() async {
    socket.emit(MessageType.stopGeofencing.name);
  }

  void stopDrive() async {
    socket.emit(MessageType.stopDrive.name);
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
