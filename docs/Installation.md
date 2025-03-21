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
docker exec -it drive_ros bash -ic "ros2 run drive_ros keyboard_teleop"
```

## Running tests

```sh
python -m pytest
```

## Running code coverage

```sh
coverage run --source=DRIVE_AGAIN -m pytest
```

and then run

```sh
coverage report
```

or alternatively, for a nicer result, run

```sh
coverage html
```

and open the generated HTML file

> Note that in order for changes in the base code to be applied, you will need to run `pip install .` between test runs.
