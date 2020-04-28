# ds4_driver

[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/naoki-mizuno/ds4_driver.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/naoki-mizuno/ds4_driver/context:python)

DualShock 4 driver for ROS.

## Features

- Get information such as IMU, battery, and touchpad from your DualShock 4.
- Use feedback such as rumble, LED color, and LED flash via ROS topics.
- Connect to your controller via Bluetooth.
- Utility node included to publish velocity commands from inputs

## Installation and Usage

This driver depends on `ds4drv`. Some features of this driver depend on pull
requests have not yet been merged upstream. Until they are merged, use
[`naoki-mizuno/ds4drv`](https://github.com/naoki-mizuno/ds4drv/tree/devel)
(`devel` branch).

```console
$ git clone https://github.com/naoki-mizuno/ds4drv --branch devel
$ cd ds4drv
$ mkdir -p ~/.local/lib/python2.7/site-packages
$ python2 setup.py install --prefix ~/.local
$ sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

Compile and source this package just like any other ROS package. To run,

```console
$ roslaunch ds4_driver ds4_driver.launch
# Or
$ rosrun ds4_driver ds4_driver_node.py
```

### Disable touchpad input device

Note: You can skip this section if you use the forked version of `ds4drv`
(i.e. `naoki-mizuno/ds4drv`) because the following line is included in the
udev rules by default.

By default the touchpad of the DualShock 4 is recognized as an input device.
Because of this, the mouse moves to the location on screen that corresponds to
the location touched, making it very hard to track the mouse cursor (and
worse, it automatically clicks at that location). If you want to prevent the
touchpad from being recognized as an input device, add the following to the
udev rules and run the `udevadm` commands (you will still be able to use the
touchpad from this driver):

```
SUBSYSTEM=="input", ATTRS{name}=="*Wireless Controller Touchpad", RUN+="/bin/rm %E{DEVNAME}", ENV{ID_INPUT_JOYSTICK}=""
```

## Demonstration

Get a glimpse of some of the features of `ds4_driver` including touchpad,
rumble, and LED control:

```
$ roslaunch ds4_driver demo.launch
```

Moving the left/right stick controls the rumble. Sliding left and right on the
touchpad while pressing circle, triangle, cross buttons controls the
brightness of the red, green, blue LED, respectively (you can tell from the
color of the button).  Pressing the PS button triggers the flashing of the
LED.

## ds4_driver_node.py

This is the main node that interacts with DualShock 4.

### Parameters

- `~device_addr`: hardware address of the device. If unspecified, will use the
  first device found.
- `~backend` (default: `hidraw`): what backend to use to connect to the device
  via Bluetooth. `bluetooth` is only there for legacy support and has not been
  tested. Refer to the docs in `ds4drv` for details.
- `~use_standard_msgs` (default: `false`): use ROS-standard messages such as
  `sensor_msgs/Joy`.
- `~autorepeat_rate` (default: `0` (disabled)): rate in Hz at which a joystick
  that has a non-changing state will resend the previously sent message. This
  parameter is only effective when `use_standard_msgs` is `true`.
- `~deadzone` (default: 0.1): amount by which the joystick has to move before
  it is considered to be off-center.
- `~frame_id`: (default: `ds4`): frame ID to be used for the messages.
- `~imu_frame_id` (default: `ds4_imu`): frame ID to be used for the IMU
  messages.

### Topics

#### Published

- `/status` (`ds4_driver/Status`): current state of the device.

#### Subscribed

- `/set_feedback` (`ds4_driver/Feedback`): feedback for the device such as
  LED color, rumble, and LED flash.

Note: To disable flash, send message with `set_led_flash: true` and
`led_flash_off: 0`.


### Topics (when `use_standard_msgs` is `true`)

#### Published

- `/raw_report` (`ds4_driver/Report`): raw, uninterpreted report that the device
  sends.
- `/battery` (`sensor_msgs/BatteryState`): battery state of the device.
- `/joy` (`sensor_msgs/Joy`): joypad state.
- `/imu` (`sensor_msgs/Imu`): IMU state.

#### Subscribed

- `/set_feedback` (`sensor_msgs/JoyFeedbackArray`): feedback for the device.

## ds4_twist_node.py

A node to convert joypad inputs to velocity commands is included in this
package. This node is something similar to
[`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy) but is specifically
for a DualShock 4.

### Parameters

- `~stamped` (default: `false`): whether to publish `Twist` or `TwistStamped`
  for the output velocity command. For robots such as TurtleBot, Husky, and
  PR2 `/cmd_vel` is not stamped (i.e.  `Twist` is used) but stamped velocity
  commands may be required for some applications.
- `~inputs`: what buttons and axes to use for the value of each velocity
  vector. Expressions can be used to combine values of multiple keys (see
  `config/twist_6dof.yaml` for examples).
- `~scales`: scaling factor for each velocity vector.

### Topics

#### Published

- `/cmd_vel` (`geometry_msgs/Twist` or `geometry_msgs/TwistStamped`): velocity
  command.

#### Subscribed

- `/status` (`ds4_driver/Status`): joypad state.

## License

MIT

## Author

Naoki Mizuno (naoki.mizuno.256@gmail.com)
