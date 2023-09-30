# EMIRO Autonomous Copter

Open Source Library for Autonomous Drone.
|   |  |
| ------------- | ------------- |
| Created 		| 30 September 2023  	|
| Finished 		| -  					|

## About
EMIRO is EEPIS Multirotor Research team that work under the auspices of Dirgantara [PENS](https://www.pens.ac.id/). This research team mainly focused on Vertical Take Off Landing research. EMIRO active following some VTOL competition especially KRTI (Kontes Robot Terbang Indonesia).</br>
**More :**</br> 
> [![LinkedIn](https://img.shields.io/badge/LinkedIn-%230077B5.svg?logo=linkedin&logoColor=white)](https://id.linkedin.com/company/emiro-pens) 


## Dependencies
1. [Ardupilot & MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials)
2. [Gazebo & ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md) (*OPTIONAL*)
3. ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)(*recommended*) / [Neotic](http://wiki.ros.org/noetic/Installation/Ubuntu) & [MAVROS](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros.md)
4. [OpenCV](https://github.com/opencv/opencv/tree/5.x)
. [JetsonGPIO](https://github.com/pjueon/JetsonGPIO) (Jetson) or [WiringPI](https://www.digikey.com/en/maker/blogs/2019/how-to-use-gpio-on-the-raspberry-pi-with-c) (Rasberry Pi)
5. [Boost](https://stackoverflow.com/questions/12578499/how-to-install-boost-on-ubuntu)
6. [JSON C++](https://github.com/open-source-parsers/jsoncpp) (jsoncpp)
7. Curses
8. [Curl](https://www.cyberciti.biz/faq/how-to-install-curl-command-on-a-ubuntu-linux)

## Install
```
cd ~/catkin_ws/src
git clone https://github.com/Balisa16/EMIRO.git emiro
catkin build emiro
```