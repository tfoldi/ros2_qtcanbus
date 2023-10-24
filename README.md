# ros2_qtcanbus
QT5 QSerialBus / QCanBus support for ROS2

This package is designed to bridge the gap between CAN buses and ROS2 by publishing CAN bus data through QSerialBus / QCanBus interfaces (including Qt5 QSerial plugins). It has a module for decoding messages (ros2_candecode) and share them as `Twist`, `NavSatFix` or `DiagnosticArray`.

## Features

- Read data from CAN buses via QSerialBus / QCanBus interfaces.
- Publish data to ROS2 topics.
- Seamless integration with existing ROS2 setups.

## Dependencies

- ROS2 Foxy, Galactic, or Rolling (or later). `main` should work on all of these.
- Qt 5.12 or later

## Installation

1. Clone the repository into your ROS2 workspace:
```bash

git clone https://github.com/tfoldi/ros2_qtcanbus.git

```

2. Build the package using colcon:

```bash
colcon build
```

## Usage
1. Source your ROS2 workspace:
```bash
source install/setup.bash
```

2. Run the CAN bus data publisher node:
```bash
ros2 run ros2_qtcanbus qtcanbus_sender --ros-args -p canbus_plugin:=<your_qt_plugin> -p canbus_interface:=<your_can_if>
```

3. Run the decode node:
```bash
ros2 launch ros2_candecode standalone.py
```

## Configuration

You can configure the decoder mappings and rules by modifying the `ros2_candecode/config/candecode.yaml` file to match your CAN bus setup. 

## Contributing
We welcome contributions, PRs are welcome.

## License
This project is licensed under the BSD 3-clause.
