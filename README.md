# m300_ws

## Dependencies
```bash
sudo apt install ros-noetic-rosserial-msgs

pip3 install pyserial
```

## Usage
```
rosrun arduino_actuator serial_node.py /dev/ttyUSB0

rosrun arduino_actuator servo_pub.py
```