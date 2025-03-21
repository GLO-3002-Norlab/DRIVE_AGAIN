# Installation

## Testing on your computer

If you want to test DRIVE on your computer before using it on a real robot, follow these steps:

1. Clone the repo

```bash
git clone git@github.com:GLO-3002-Norlab/DRIVE_AGAIN.git
```

2. Build the docker images

```bash
cd DRIVE_AGAIN
docker compose build
```

3. Launch the application

```bash
docker compose up -d
```

4. Go on [http://localhost:5000](http://localhost:5000), you should see the user interface
5. Launch a teleoperation node to control the virtual robot

```bash
docker exec -it drive_ros bash -ic "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```
