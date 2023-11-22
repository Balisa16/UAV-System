import subprocess

class TextColor:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    ITALIC = '\033[3m'
    END = '\033[0m'

class Checker:
    """
    Checker class for pre-flight
    """
    def __init__(self) -> None:
        print("Pre-Flight Checker")
        pass

    def lidar_pub(self):
        print(TextColor.BLUE + "Lidar Publish Check" + TextColor.END)
        command = "rostopic list"
        lidar_sub = "/scan"
        try:
            output = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            str1 = str(output.stdout).replace("b'", "")
            str1 = str1.replace("\\n'", "")
            comp = str(str1).split("\\n")
            if lidar_sub in comp:
                print(f"Lidar is already Published")
                return True
        except :
            print(f"Failed Check Lidar Publisher")
        print(f"Lidar is NOT Published")
        return False
    
    def available_cam(self, need = 0):
        print(TextColor.BLUE + "Camera Check" + TextColor.END)
        command = "ls /dev/video*"
        is_find = False
        try:
            output = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            str1 = str(output.stdout).replace("b'", "")
            str1 = str1.replace("\\n'", "")
            comp = str(str1).split("\\n")
            for idx, cam in enumerate(comp):
                i = cam.split("/dev/video")
                if int(i[1]) == need:
                    is_find = True
                print(f"{idx}: Cam {i[1]}\t: {cam}")
                
        except :
            print(f"Failed Check Camera")
        return is_find
    
    def position(self, tolerance = 0.04):
        print(TextColor.BLUE + "Copter Position Check" + TextColor.END)
        command = "rostopic echo -n 1 /mavros/local_position/pose/pose/position"
        in_range = False
        try:
            highest = 0.000
            output = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            str1 = str(output.stdout).replace("b'", "")
            str1 = str1.replace("\\n'", "")
            comp = str(str1).split("\\n")
            for idx, pose in enumerate(comp):
                print(pose)
                temp = pose.split(":")
                val_abs = abs(float(temp[1]))
                if val_abs > highest:
                    highest = val_abs
                if idx >= 2:
                    break
            print("Resume :")
            print(f"\tTolerance\t: {format(tolerance*100, '.2f')} cm")
            print(f"\tHighest\t\t: {format(highest*100, '.2f')} cm")
            if highest >= tolerance :
                print(TextColor.RED + "Position offset too high" + TextColor.END)
            else:
                in_range = True
                print(TextColor.GREEN + "Position offset in tolerance range" + TextColor.END)
        except :
            print(f"Failed Check Position")
        return in_range


if __name__ == "__main__":
    checker = Checker()
    health_score = 0
    health_score += 1 if checker.lidar_pub() else 0
    health_score += 1 if checker.available_cam(0) else 0
    health_score += 1 if checker.position(0.05) else 0

    if health_score is 3:
        print(TextColor.BLUE + TextColor.BOLD + "\nReady to Flight." + TextColor.END)
    elif health_score is 2:
        print(TextColor.YELLOW + TextColor.BOLD + "\nNot Ready to Flight." + TextColor.END)
    else:
        print(TextColor.RED + TextColor.BOLD + "\nNot Ready to Flight." + TextColor.END)

