#!/usr/bin/bash

install_ubuntu20()
{
    source child/ubuntu20.sh

    init_dir

    echo "Installing System on Ubuntu 20.04"

    echo -e "\n\n\033[1;32m>>> Updating Ubuntu\033[0m"
    sudo apt update
    sudo apt upgrade -y
    
    echo -e "\n\n\033[1;32m>>> Installing CMake and Git\033[0m"
    sudo apt install -y cmake git

    ins_ardupilot_mavproxy
}

install_ubuntu18()
{
    echo "Installing System on Ubuntu 18.04"
}


# Check if the script is being run with sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo privileges. Please run with sudo."
    exit 1
fi

# Check if the /etc/os-release file exists
if [ -e /etc/os-release ]; then
    # Source the file to get the variables
    . /etc/os-release

    # Check if the variable ID is set and if it is "ubuntu"
    if [ "$ID" == "ubuntu" ]; then
        # Check the version
        if [ "$VERSION_ID" == "18.04" ]; then
            echo "Ubuntu 18.04 detected."
            install_ubuntu18
        elif [ "$VERSION_ID" == "20.04" ]; then
            echo "Ubuntu 20.04 detected."
            install_ubuntu20
        else
            echo "Ubuntu version other than 18.04 or 20.04 detected."
        fi
    else
        echo "Not an Ubuntu distribution."
    fi
else
    echo "Unable to determine the distribution."
fi
