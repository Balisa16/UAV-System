#! /bin/bash
source ~/.bashrc
echo "Running Copter Pre-Flight Check System"
python3 "${EMIRO_PATH}/check.py" -i "$1"