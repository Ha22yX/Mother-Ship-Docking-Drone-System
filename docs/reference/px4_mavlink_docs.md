# PX4 MAVLink Serial Control References

Core documentation links:

- PX4 Docs: https://docs.px4.io/
- PX4 Offboard mode: https://docs.px4.io/main/en/flight_modes/offboard.html
- MAVLink main site: https://mavlink.io/en/
- MAVLink message dictionary: https://mavlink.io/en/messages/common.html

Useful MAVLink entries:

- MAV_CMD list: https://mavlink.io/en/messages/common.html#mav_cmd
- SET_POSITION_TARGET_LOCAL_NED: https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
- SET_ATTITUDE_TARGET: https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET

Notes:

- Serial control uses MAVLink over UART.
- Offboard requires continuous setpoints (typically > 2 Hz), or it will exit.

