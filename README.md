# m300_ws

## Dependencies
```bash
sudo apt install ros-noetic-rosserial-msgs

pip3 install pyserial
```

## Usage
```
# to run the fire localization and SLAM
rosrun ORB_SLAM3 fire_localization /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Examples_old/Monocular/GoPro.yaml

# to run the fire detection
rosrun forest_fire_geopositioning fire_detection_v8.py

# to run the geopositioning
rosrun forest_fire_geopositioning geo_positioning

# to release the fire extinguisher
rosrun arduino_actuator serial_node.py /dev/ttyUSB0
rosrun arduino_actuator servo_pub.py

# to visualize the fire bounding boxes
rosrun forest_fire_geopositioning fire_spots_visualization

# to launch all nodes
roslaunch forest_fire_geopositioning fire_geopositioning.launch
```

## Dateset available
https://drive.google.com/file/d/1YPX3RgdjjUx_tRMU9sRcODnS2XP0VS_U/view?usp=sharing

### Topics
```
/bounding_boxes/fire_spots
/clock
/dji_osdk_ros/gps_position
/dji_osdk_ros/main_wide_RGB
/position/camera_pose
/position/fire_spots
/position/fire_spots_GPS
/position/real_scale
```