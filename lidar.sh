echo "1" | sudo -S chmod 666 /dev/ttyUSB0
if [ ! -z "$1" ]
  then
    roslaunch rplidar_ros rplidar_$1.launch
  else
  	echo "Missing parameter. Using S1 lidar."
  	roslaunch rplidar_ros rplidar_s1.launch
fi