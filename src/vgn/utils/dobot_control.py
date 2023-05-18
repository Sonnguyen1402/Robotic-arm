import socket
import re
from time import sleep
import sys
import rospy

# import thread module
from _thread import *
import threading


class Control_Dobot_TCP (object):
    pose = [150, -240, 450, 179, -0.0, 179]
    sended_flag = False
    timeout_counter = 0
    command_name = ""
    command_executed = False

    def __init__(self, address):
        try: 
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect(address)
            self.s.settimeout(2)
            rospy.loginfo('Connect to Robot Dobot successfully')
            self.print_lock = threading.Lock()
        except socket.error as e:
            rospy.loginfo (e)
            sys.exit("Fail to connect Robot!")

    def TCP_Close(self):
        self.s.close()

    def convert_msg(self, msg):
        try:
            data = msg.split(",")
            if len(data) == 6:
                floats_list = [float(x) for x in data]
                robot_pose = [round(elem, 4) for elem in floats_list]
                return robot_pose
        except:
            return False

    def send_command(self, command):
        try:
            self.s.sendall(bytes(command, "utf8"))
            rospy.loginfo ("Send command %s to Robot", self.command_name)
            self.sended_flag = True
            self.command_executed = False
        except socket.error as e:
            rospy.loginfo (e)
            rospy.loginfo ("Fail to send command to Robot!")

    def get_response(self):
        while True:
            try:
                msg = self.s.recv(1024)
            except socket.timeout as e:
                err = e.args[0]
                if err == 'timed out':
                    if self.sended_flag:
                        if self.timeout_counter <= 3:
                            self.timeout_counter += 1
                        else:
                            rospy.loginfo ("Timeout, Fail to comfirm Robot action")
                            self.timeout_counter = 0
                            self.sended_flag = False
                    continue
                else:
                    rospy.loginfo (err)
                    break
            except socket.error as e:
                rospy.loginfo (e)
                break
            else:
                if len(msg) == 0:
                    rospy.loginfo ('Robot disconnected')
                    self.TCP_Close()
                    break
                else:
                    utf8_msg = msg.decode("utf8")
                    if self.sended_flag:
                        if self.command_name == "O" and utf8_msg == "Opened":
                            rospy.loginfo  ("Opened")
                            self.command_executed = True                          
                            self.timeout_counter = 0
                            self.sended_flag = False
                        elif self.command_name == "C" and utf8_msg == "Closed":
                            rospy.loginfo  ("Closed")
                            self.command_executed = True
                            self.timeout_counter = 0
                            self.sended_flag = False
                        elif self.command_name == "S" and utf8_msg == "Grasped":
                            rospy.loginfo  ("Grasped")
                            self.command_executed = True
                            self.timeout_counter = 0
                            self.sended_flag = False
                        elif self.command_name == "S" and utf8_msg == "Empty":
                            rospy.loginfo  ("Empty")
                            # self.command_executed = True
                            self.sended_flag = False
                            self.timeout_counter = 0
                        elif self.command_name == "M":
                            robot_pose = self.convert_msg(utf8_msg)
                            if not robot_pose == False:
                                newpose = [round(elem, 4) for elem in self.pose]
                                if robot_pose == newpose:
                                    rospy.loginfo  ("Move completed")
                                    self.command_executed = True
                                    self.timeout_counter = 0
                                    self.sended_flag = False
                                else:
                                    rospy.loginfo ("Move failed")

    def goto_pose(self, pose):
        if not self.sended_flag:
            self.pose = pose
            strpose = re.sub("\[|\]", "", str(self.pose))
            self.command_name = "M"
            self.send_command("M, " + strpose)
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
        
    def move_home(self):
        if not self.sended_flag:
            self.pose = [150, -240, 450, 179, -0.0, 179]
            strpose = re.sub("\[|\]", "", str(self.pose))
            self.command_name = "M"
            self.send_command("M, " + strpose)
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
            
    def open_gripper(self):
        if not self.sended_flag:
            self.command_name = "O"
            self.send_command("O")
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
        
    def close_gripper(self):
        if not self.sended_flag:
            self.command_name = "C"
            self.send_command("C")
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False

    def status_gripper(self):
        if not self.sended_flag:
            self.command_name = "S"
            self.send_command("S")
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False

    def threaded(self):
        self.get_response()
        self.print_lock.release()
        rospy.loginfo ("Listener destroyed")
        sys.exit(0)
            
    def run_listener(self):
        self.print_lock.acquire()
        start_new_thread(self.threaded, ())