#! /bin/bash

if [ ! -z "$2" ]
  then
    echo "Open Servo on Aux Out : $2"
	rosrun mavros mavcmd long 183 $2 $1 0 0 0 0 0
  elif [ ! -z "$1" ]; then
    for i in {1..4}
	do
	    echo "Open Servo on Aux Out : $i"
		rosrun mavros mavcmd long 183 $i $1 0 0 0 0 0
	done
  else
    echo -e "Incorrect argument.\nOption 1 :\n\t./open_servo.sh <servo_pwm>\nOption 2 :\n\t./open_servo.sh <servo_pwm> <aux_index>"
fi