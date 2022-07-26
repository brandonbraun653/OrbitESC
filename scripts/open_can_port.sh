# Configures the CAN0 interface for 1MBaud. Must be run with root privileges.
modprobe can
modprobe can-raw
ip link set can0 type can bitrate 1000000
ip link set can0 up