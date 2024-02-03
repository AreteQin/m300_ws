This package depends on all the other three packages and ROS vision message
```bash
sudo apt install ros-noetic-vision-msgs
```

## Usage
```bash
source ~/m300_ws/devel/setup.bash
rosrun forest_fire_geopositioning fire_detection.py
rosrun ORB_SLAM3 fire_localization /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Vocabulary/ORBvoc.txt /home/qin/Downloads/ORB_SLAM3_Ubuntu_20/Examples_old/Monocular/GoPro.yam
```