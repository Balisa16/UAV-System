import subprocess
import getpass
import argparse
from datetime import datetime
import time
import pyautogui
import psutil
import socket
import os
import signal

class SSHTerminalManager:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.password = getpass.getpass("SSH Password: ")
        self.terminal_inc = 1

    def is_ssh_reachable(self, host, port=22, timeout=3):
        try:
            socket.create_connection((host, port), timeout=timeout)
            print(f"Status {host} : OK")
            return True
        except (socket.timeout, ConnectionRefusedError):
            print(f"Status {host} : FAILED")
            return False
        except :
            print(f"Status {host} : NOT IN NETWORK")
            return False
            

    def open_mp(self):
        print("Opening Mission Planner")
        process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'cd ~ && ./mp.sh'])
        time.sleep(6)
        pyautogui.click(1840, 78)
        pyautogui.press('up')
        pyautogui.press('up')
        pyautogui.press("enter")
        pyautogui.click(1881, 92)
        return process.pid

    def open_terminal_and_ssh_with_command(self, pid):
        # First display
        disp_word = f"Connected by ssh to\nIP:{self.terminal_inc}"

        ssh_command = f"sshpass -p '{self.password}' ssh -t {self.ip_address} 'lsb_release -a  && exec $SHELL'"
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', ssh_command])
        
        # Set terminal always on top
        pyautogui.moveTo(cmd["x_from"], cmd["y_from"])
        pyautogui.mouseDown(button='right')
        pyautogui.mouseUp(button='right')
        for i in range(5):
            pyautogui.press('down')
        pyautogui.press("enter")

        # Move terminal
        pyautogui.moveTo(cmd["x_from"], cmd["y_from"])
        pyautogui.mouseDown()
        pyautogui.moveTo(cmd["x_to"], cmd["y_to"])
        pyautogui.mouseUp()

        pyautogui.click(cmd["x_to"], cmd["y_to"] + 50)
        pyautogui.typewrite(cmd["command"])
        if self.terminal_inc == 1:
            pyautogui.press("enter")
        self.terminal_inc += 1
    
    def terminate_mp(self, pid):
        print("Disconnect Mission Planner")
        pyautogui.click(1881, 92)
        time.sleep(1)

        print("Kill Mission Planner")
        pyautogui.click(1902, 46)

    
    def press_enter(self, cmd):
        pyautogui.click(cmd["x_to"], cmd["y_to"] + 50)
        if not cmd["waiting"] and self.terminal_inc != 1:
            pyautogui.press("enter")
        self.terminal_inc += 1

    def move_current_terminal(self, x, y):
        command = f"xdotool search --onlyvisible --class gnome-terminal windowactivate windowfocus && xdotool getactivewindow windowmove {x} {y} && xdotool getactivewindow windowsize 560 280"
        subprocess.run(command, shell=True)
        print("Window moved")

    def update_time_by_ssh(self, t_out):
        current_time = datetime.now()
        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
        date_command = f'echo "{self.password}" | sudo -S date -s "{formatted_time}"'
        ssh_command_date = f"sshpass -p '{self.password}' ssh -t {self.ip_address} '{date_command}; exit; exec $SHELL'"
        date_update_result = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', ssh_command_date])
        print(f"Update time running in PID {date_update_result.pid}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="A script to work with IP addresses.")
    parser.add_argument('-i', '--ip', required=True, help='The IP address to work with')
    args = parser.parse_args()
    ip_address = args.ip

    terminal_manager = SSHTerminalManager(ip_address)

    # Move current terminal
    terminal_manager.move_current_terminal(1, 850)

    # Opening Mission Planner
    mp_pid = terminal_manager.open_mp()

    isEnter = input("Press enter to continued Opening System Copter terminal.")

    just_ip = ip_address.split("@")
    client_reached = terminal_manager.is_ssh_reachable(just_ip[1])

    if client_reached is True:
        terminal_manager.update_time_by_ssh(5)
        commands_list = [
            # {
            #     "command": "./rs.sh",
            #     "waiting": False,
            #     "x_from": 320,
            #     "y_from": 140,
            #     "x_to": 1645,
            #     "y_to": 160
            # }
            # ,
            # {
            #     "command": "./apm.sh",
            #     "waiting": False,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 1645,
            #     "y_to": 400
            # },
            # {
            #     "command": "roslaunch rplidar_ros rplidar_s1.launch",
            #     "waiting": False,
            #     "pass": False,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 1111,
            #     "y_to": 880
            # },
            # {
            #     "command": "./rs2.sh",
            #     "waiting": True,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 1645,
            #     "y_to": 640
            # },
            # {
            #     "command": "rosrun emiro refactor 1 2.0 6.0 5.0 0 1",
            #     "waiting": True,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 1645,
            #     "y_to": 880
            # },
            # {
            #     "command": "date && ls /dev/video* && rosservice call /mavros/set_stream_rate 0 100 1",
            #     "waiting": True,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 1111,
            #     "y_to": 640
            # },
            # {
            #     "command": "rostopic echo /mavros/local_position/pose",
            #     "waiting": True,
            #     "x_from": 270,
            #     "y_from": 85,
            #     "x_to": 240,
            #     "y_to": 640
            # }
        ]


        # Open terminals and SSH into each IP address with the command
        print("Running Copter System terminal")
        for cmd in commands_list:
            terminal_manager.open_terminal_and_ssh_with_command(cmd)
        
        terminal_manager.terminal_inc = 1
        for cmd in commands_list:
            terminal_manager.press_enter(cmd)
    else:
        print("Failed reached client.")
        terminal_manager.terminate_mp(mp_pid)
