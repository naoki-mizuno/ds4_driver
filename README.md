# ds4_ros

DualShock 4 driver for ROS.

## Features

- Get information such as IMU, battery, and touchpad from your DualShock 4.
- Use feedback such as rumble, LED color, and LED flash via ROS topics.
- Connect to your controller via Bluetooth.

## Usage

Download the latest `ds4drv` from GitHub. Some features of `ds4_ros` depend on
pull requests have not yet been merged, so until then, use
[`naoki-mizuno/ds4drv`](https://github.com/naoki-mizuno/ds4drv/tree/devel)
(`devel` branch).

```console
$ git clone https://github.com/naoki-mizuno/ds4drv --branch devel
$ cd ds4drv
$ python2 setup.py install --prefix ~/.local
$ sudo cp 50-ds4drv.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

Compile and source this package just like any other ROS package. To run,

```console
$ roslaunch ds4_ros ds4_ros.launch
# Or
$ rosrun ds4_ros ds4_ros_node.py
```

## Parameters

- `~device_addr`: hardware address of the device. If unspecified, will use the
  first device found.
- `~backend` (default: `hidraw`): what backend to use to connect to the device
  via Bluetooth. `bluetooth` is only there for legacy support and has not been
  tested. Refer to the docs in `ds4drv` for details.
- `~use_standard_msgs` (default: `false`): use ROS-standard messages such as
  `sensor_msgs/Joy`.
- `~deadzone` (default: 0.05): amount by which the joystick has to move before
  it is considered to be off-center.
- `~frame_id`: (default: `ds4`): frame ID to be used for the messages.
- `~imu_frame_id` (default: `ds4_imu`): frame ID to be used for the IMU
  messages.

## Topics

### Published

- `/status` (`ds4_ros/Status`): current state of the device.

### Subscribed

- `/set_feedback` (`ds4_ros/Feedback`): feedback for the device such as
  LED color, rumble, and LED flash.

Note: To disable flash, send message with `set_led_flash: true` and
`led_flash_off: 0`.


## Topics (when `use_standard_msgs` is `true`)

### Published

- `/raw_report` (`ds4_ros/Report`): raw, uninterpreted report that the device
  sends.
- `/battery` (`sensor_msgs/BatteryState`): battery state of the device.
- `/joy` (`sensor_msgs/Joy`): joypad state.
- `/imu` (`sensor_msgs/Imu`): IMU state.

### Subscribed

- `/set_feedback` (`sensor_msgs/JoyFeedbackArray`): feedback for the device.

## License

MIT

## Author

Naoki Mizuno (naoki.mizuno.256@gmail.com)
