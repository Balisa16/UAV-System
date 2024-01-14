check(){
    echo "Checking EMIRO :"
    printf "Copter Directory "
    if [ -d "$EMIRO_PATH/copter" ]; then
        printf "\033[1m\033[32m(OK)\033[0m\n"
        if [ ! -e "$EMIRO_PATH/copter/plan.json" ]; then
            touch "$EMIRO_PATH/copter/plan.json"
            echo  -e "[\n\t{\n\t\t\"header\": \"Takeoff\",\n\t\t\"speed\": 1.00,\n\t\t\"x\": 0.00,\n\t\t\"y\": 0.00,\n\t\t\"z\": 1.00,\n\t\t\"yaw\": 90.00\n\t}\n]" >> $EMIRO_PATH/copter/plan.json
        fi
    else
        printf "\033[1m\033[31m(Failed)\033[0m -> Created\n"
        mkdir -p "$EMIRO_PATH/copter"
        touch "$EMIRO_PATH/copter/plan.json"
        echo  -e "[\n\t{\n\t\t\"header\": \"Takeoff\",\n\t\t\"speed\": 1.00,\n\t\t\"x\": 0.00,\n\t\t\"y\": 0.00,\n\t\t\"z\": 1.00,\n\t\t\"yaw\": 90.00\n\t}\n]" >> $EMIRO_PATH/copter/plan.json
    fi
}
sitl()
{
	. ~/sitl.sh
}
scan()
{
	roslaunch iq_sim scan.launch
}
apm()
{
	roslaunch iq_sim apm.launch
}