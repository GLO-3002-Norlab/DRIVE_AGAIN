# DRIVE_AGAIN

# Setup UI
## Linux
1. Install Flutter dependencies 
```sh
sudo apt-get update -y && sudo apt-get upgrade -y;
sudo apt-get install -y curl git unzip xz-utils zip libglu1-mesa build-essential libgtk-3-dev
```
2. Install dart extension on vscode
3. Install Flutter extension on vscode
4. Add Flutter to path
```sh
export PATH="${PATH_TO_FLUTTER}/flutter/bin:${PATH}"
```

## Windows (VSCode)
1. Install [Flutter Extension](https://marketplace.visualstudio.com/items?itemName=Dart-Code.flutter).
2. Follow on-screen prompts to install [Dart Entension](https://marketplace.visualstudio.com/items?itemName=Dart-Code.dart-code).
3. In the bottom-right, click on `Device` then select `Enable Chrome for this project`.
4. Execute `main.dart`.

# Setup project

1. Create venv with python 3.10
2. Install dependencies

```sh
pip install -e .
```

## Test

```sh
python -m pytest
```

## Run sim

```sh
python src/DRIVE_AGAIN/sim.py # Keyboard teleop mode
python src/DRIVE_AGAIN/sim.py drive # Drive protocol
```
