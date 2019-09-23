# ds4_driver

DualShock 4 driver for ROS.

## Features

- Get information such as IMU, battery, and touchpad from your DualShock 4.
- Use feedback such as rumble, LED color, and LED flash via ROS topics.
- Connect to your controller via Bluetooth.

## Usage

This driver depends on `ds4drv`. Some features of this driver depend on pull
requests have not yet been merged upstream. Until they are merged, use
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

Note: If you want to prevent the touchpad from being recognized as an input
device, add the following to the udev rules and run the `udevadm` commands
(you will still be able to use the touchpad from this driver):

```
SUBSYSTEM=="input", ATTRS{name}=="*Wireless Controller Touchpad", RUN+="/bin/rm %E{DEVNAME}", ENV{ID_INPUT_JOYSTICK}=""
```

Compile and source this package just like any other ROS package. To run,

```console
$ roslaunch ds4_driver ds4_driver.launch
# Or
$ rosrun ds4_driver ds4_driver_node.py
```

## Parameters

- `~device_addr`: hardware address of the device. If unspecified, will use the
  first device found.
- `~backend` (default: `hidraw`): what backend to use to connect to the device
  via Bluetooth. `bluetooth` is only there for legacy support and has not been
  tested. Refer to the docs in `ds4drv` for details.
- `~use_standard_msgs` (default: `false`): use ROS-standard messages such as
  `sensor_msgs/Joy`.
- `~pub_joy_on_change` (default: `true`): only publish Joy messages when
  changes to axes or buttons is detected. This parameter is only effective
  when `use_standard_msgs` is `true`.
- `~deadzone` (default: 0.05): amount by which the joystick has to move before
  it is considered to be off-center.
- `~frame_id`: (default: `ds4`): frame ID to be used for the messages.
- `~imu_frame_id` (default: `ds4_imu`): frame ID to be used for the IMU
  messages.

## Topics

### Published

- `/status` (`ds4_driver/Status`): current state of the device.

### Subscribed

- `/set_feedback` (`ds4_driver/Feedback`): feedback for the device such as
  LED color, rumble, and LED flash.

Note: To disable flash, send message with `set_led_flash: true` and
`led_flash_off: 0`.


## Topics (when `use_standard_msgs` is `true`)

### Published

- `/raw_report` (`ds4_driver/Report`): raw, uninterpreted report that the device
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
