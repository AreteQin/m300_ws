# m300_ws

## Dependencies
```bash
sudo apt install ros-noetic-rosserial-msgs

pip3 install pyserial
```

## Usage
```
rosrun ORB_SLAM3 fire_localization /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Examples_old/Monocular/GoPro.yaml

rosrun arduino_actuator serial_node.py /dev/ttyUSB0

rosrun arduino_actuator servo_pub.py
```