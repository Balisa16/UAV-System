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
    
    def run_terminal_app(self):
        terminal_app_path = os.environ.get('TERMINAL_APP', '')
        if terminal_app_path:
            subprocess.run("./main2", shell=False, cwd=terminal_app_path)
        else:
            print("TERMINAL_APP environment variable not set.")

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
        print("Running Copter System Terminal.")
        terminal_manager.run_terminal_app()

    else:
        print("Failed Reached Client.")
        terminal_manager.terminate_mp(mp_pid)
