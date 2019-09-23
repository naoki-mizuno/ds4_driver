# Human-readable and more ROS-compatible status of the device
Header header

# Stick
# Left: 1.0, Right: -1.0
float32 axis_left_x
# Up: 1.0, Down: -1.0
float32 axis_left_y
float32 axis_right_x
float32 axis_right_y

# Shoulder buttons [0, 1.0]
float32 axis_l2
float32 axis_r2

# Buttons (0: Not pressed, 1: Pressed)
int32 button_dpad_up
int32 button_dpad_down
int32 button_dpad_left
int32 button_dpad_right
int32 button_cross
int32 button_circle
int32 button_square
int32 button_triangle
int32 button_l1
int32 button_l2
int32 button_l3
int32 button_r1
int32 button_r2
int32 button_r3
int32 button_share
int32 button_options
int32 button_trackpad
int32 button_ps

# Gyro
sensor_msgs/Imu imu

# Battery
float32 battery_percentage  # [0.0, 1.0]
int32 battery_full_charging # 0: No, 1: Yes

# Trackpads
ds4_driver/Trackpad touch0
ds4_driver/Trackpad touch1

# Plugs
int32 plug_usb              # 0: No, 1: Yes
int32 plug_audio            # 0: No, 1: Yes
int32 plug_mic              # 0: No, 1: Yes
