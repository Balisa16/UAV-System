#!/usr/bin/bash

init_dir()
{
    echo -e "\n\n\033[1;32m>>> Creating Copter Directory\033[0m"
    mkdir -p ~/drone
}

ins_ardupilot_mavproxy()
{
    echo -e "\n\n\033[1;32m>>> Installing Ardupilot and MAVProxy\033[0m"
    cd ~/drone
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile

    # Checkout into the Copter branch
    git checkout Copter-4.4
    git submodule update --init --recursive

    cd ~/drone/ardupilot/ArduCopter
    sim_vehicle.py -w
}

ins_gazebo()
{
    echo -e "\n\n\033[1;32m>>> Installing Gazebo\033[0m"

    # Setup Source and Keys
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt update

    # Install Gazebo
    sudo apt-get install -y gazebo11 libgazebo11-dev
    cd ~/drone
    git clone https://github.com/khancyr/ardupilot_gazebo.git
    cd ardupilot_gazebo

    #  Build and Install
    mkdir build
    cd build
    cmake ..
    make -j$(( $(nproc) - 1 )) # Don't use full threads
    sudo make install

    # Register Gazebo
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

    # Set Gazebo Model Path
    echo 'export GAZEBO_MODEL_PATH=~/drone/ardupilot_gazebo/models' >> ~/.bashrc
    . ~/.bashrc


}

ins_ros()
{
    echo -e "\n\n\033[1;32m>>> Installing ROS\033[0m"

    # Setup Source and Keys
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install -y curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    # Update Package List
    sudo apt update

    # Install ROS Desktop Full
    sudo apt install -y ros-noetic-desktop-full

    # Register ROS path into bashrc script
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    . ~/.bashrc

    #  Install Dependencies for ROS Building Packages
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

    # Initialize rosdep
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
}

setup_workspace()
{
    echo -e "\n\n\033[1;32m>>> Setting up Workspace\033[0m"

    # Install desire dependencies
    sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
    pip3 install osrf-pycommon


    mkdir -p ~/drone/ws/src
    cd ~/drone/ws
    catkin init

    cd ~/drone/ws
    wstool init ~/drone/src

    rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
    rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
    wstool merge -t src /tmp/mavros.rosinstall
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

    catkin build

    # Register Catkin devel path
    echo "source ~/drone/ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    # Install Geographiclib
    sudo ~/drone/ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

    
}

