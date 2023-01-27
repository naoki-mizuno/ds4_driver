# Message file for controlling DualShock4
# LED intensities [0.0, 1.0]
bool set_led
float32 led_r
float32 led_g
float32 led_b

# LED flash durations in seconds (max is 2.5 sec)
bool set_led_flash
# Note: Set led_flash_off to 0 to stop flashing
float32 led_flash_on
float32 led_flash_off

# Rumble intensities [0.0, 1.0] (continues rumbling for 5 sec by default)
bool set_rumble
float32 rumble_duration
float32 rumble_small
float32 rumble_big
