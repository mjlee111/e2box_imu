# e2box_imu

9DOF e2box imu ROS2 package.

## How to use

1. Clone this repository.
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/mjlee111/e2box_imu.git
```

2. Build this package.
```bash
$ cd ~/ros2_ws
$ colcon build --packages-select e2box_imu
```

3. Source the workspace.
```bash
$ source ~/ros2_ws/install/setup.bash
```

4. Launch this package.
```bash
$ ros2 launch e2box_imu e2box_imu_9dofv4.launch.py
```

## ROS Parameters
| Parameter Name | Type | Default Value | Description |
| -------------- | ---- | ------------- | ----------- |
| `port_name` | `string` | `/dev/ttyUSB0` | The serial port to which the device is connected. |
| `baudrate` | `int` | `115200` | The baud rate for serial communication. |
| `loop_rate` | `int` | `100` | The loop rate for publishing IMU data. |
| `angular_velocity_threshold` | `double` | `0.3` | The threshold for angular velocity. |
| `linear_acceleration_threshold` | `double` | `2.0` | The threshold for linear acceleration. |

## Contributing
I welcome all contributions! Whether it's bug reports, feature suggestions, or pull requests, your input helps me to improve. If you're interested in contributing, please check out my contributing guidelines or submit an issue.

## License
This project is licensed under the [Apache 2.0 License](LICENSE). Feel free to use and distribute it according to the terms of the license.

## Contact
If you have any questions or feedback, don't hesitate to reach out! You can contact me at [menggu1234@naver.com][email].

[email]: mailto:menggu1234@naver.com


