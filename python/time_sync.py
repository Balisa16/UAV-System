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

    just_ip = ip_address.split("@")
    terminal_manager.update_time_by_ssh(5)