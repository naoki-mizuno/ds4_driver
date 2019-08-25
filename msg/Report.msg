# Raw report from DualShock 4
Header header

# Left: 0, Right: 255
uint8 left_analog_x
# Up: 0, Down: 255
uint8 left_analog_y
uint8 right_analog_x
uint8 right_analog_y

# Released: 0, Pressed: 255
uint8 l2_analog
# Released: 0, Pressed: 255
uint8 r2_analog

# Released: 0, Pressed: 1
bool dpad_up
bool dpad_down
bool dpad_left
bool dpad_right
bool button_cross
bool button_circle
bool button_square
bool button_triangle
bool button_l1
bool button_l2
bool button_l3
bool button_r1
bool button_r2
bool button_r3
bool button_share
bool button_options
bool button_trackpad
bool button_ps

# IMU
int16 lin_acc_x
int16 lin_acc_y
int16 lin_acc_z
int16 ang_vel_x
int16 ang_vel_y
int16 ang_vel_z

# Top-left: (0, 0), Bottom-right: (1919, 942)
uint16 trackpad_touch0_id
uint16 trackpad_touch0_active
uint16 trackpad_touch0_x
uint16 trackpad_touch0_y
uint16 trackpad_touch1_id
uint16 trackpad_touch1_active
uint16 trackpad_touch1_x
uint16 trackpad_touch1_y

uint8 timestamp
# Full: 8, Full (and charging): 11
uint8 battery

# Unused: 0, Plugged: 1
bool plug_usb
bool plug_audio
bool plug_mic
