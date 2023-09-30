# KRTI 2023 Mission

Node for GPS

## Dependencies
### Install [Jetson GPIO](https://github.com/pjueon/JetsonGPIO)
```
git clone https://github.com/pjueon/JetsonGPIO
cd JetsonGPIO
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
Reboot
```
sudo reboot
```
Check
```
find /home -name JetsonGPIO.h
```
## Install
Uninstall ModemManager firstly
```
sudo apt-get purge modemmanager
```
### Setting swap file
See available swap
```
free -h
```
Create swap file into 4GB
```
sudo fallocate -l 4G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'
```
### Clone Project
```
cd ~/catkin_ws/src
git clone https://github.com/Balisa16/mission2023.git emiro9
```
### Build Project
```
sudo apt-get install libncurses5-dev libncursesw5-dev
cd ~/catkin_ws
catkin build
```
### Copy Shell Script
```
cp ~/catkin_ws/src/emiro9/rs.sh ~/rs.sh
cp ~/catkin_ws/src/emiro9/rs2.sh ~/rs2.sh
cp ~/catkin_ws/src/emiro9/apm.sh ~/apm.sh
cp ~/catkin_ws/src/emiro9/open_servo.sh ~/open_servo.sh
cp ~/catkin_ws/src/emiro9/close_servo.sh ~/close_servo.sh

chmod +x ~/rs.sh ~/rs2.sh ~/apm.sh ~/open_servo.sh ~/close_servo.sh
```

## Note
Vision drone is better


## Ardupilot No-GPS/GPS transition
1. [How Upload Lua File](https://ardupilot.org/copter/docs/common-lua-scripts.html)
2. [Example](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/ahrs-source.lua)
3. [Param Settings](https://ardupilot.org/rover/docs/common-non-gps-to-gps.html)
4. [Issue Lists](https://github.com/ArduPilot/ardupilot/issues/15859)
5. [Mav CMD](https://github.com/ArduPilot/ardupilot/pull/18345)


## Another Related
1. [Set Home](https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html)
2. [Command Long](https://docs.ros.org/en/hydro/api/mavros/html/srv/CommandLong.html)
3. [Example Command](https://github.com/BenbenIO/simple-Mavlink-C-rover)

## Notes
### Aux Out Configuration :
1. Paper Box 1
2. Paper Box 2
3. Paper Box 3
4. Paper Box 4

### GPIO Jetson
1. Pin 7 = Magnet
2. Pin 16 = Pulley Up Signal
2. Pin 18 = Pulley Down Signal

### Hardware Module
1. [Switch Module](https://www.tokopedia.com/permony/pc817-4-channel-optocoupler-module-4-ch-opto)

### Notes
On Test 9 need to run TF mini program when apm is already initialize
```
./tf.sh /dev/ttyTHS1 115200
```

APM Permission
```
sudo chmod 666 /dev/ttyACM0
```
Set stream rate rostopic when **/mavros/global_position/local** is not publish nothing
```
rosservice call /mavros/set_stream_rate 0 100 1
```
Terminal automation opened
Disable Wayland terminal (on ubuntu 22.04)
```
sudo nano /etc/gdm3/custom.conf
```
Scroll until find 
```
#WaylandEnable=false
```
Uncomment and save it. And than run this comamnd to refresh
```
systemctl restart gdm3
```
On Client
```
pip install argparse
sudo apt install sshpass
```
On Server
```
sudo apt install xdotool
```
## Install TF-Mini
[Documentation](https://ardupilot.org/copter/docs/common-benewake-tfmini-lidar.html)

## Install Check System in Jetson Nano
Add this line in `.bashrc`
```
export EMIRO_PATH="$HOME/catkin_ws/src/emiro"
```