import socket
import re
from time import sleep
import sys

# import thread module
from _thread import *
import threading


class Control_Dobot_TCP (object):
    pose = [138, -244, 413, 179, -0.9, 179]
    retry_counter = 0
    sended_flag = False
    timeout_counter = 0
    command_name = ""
    command_executed = False
    exit_flag = False

    def __init__(self, address):
        while True:
            try: 
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect(address)
                self.s.settimeout(3)
                # self.s.setblocking(False)
                print('Connected to Robot Arm at %s port %s' % (address[0], address[1]))
                self.retry_counter = 0
                self.print_lock = threading.Lock()
            except socket.error as e:
                print (e)
                if self.retry_counter <= 10:
                    self.retry_counter += 1
                else:
                    print("")
                    sys.exit("Fail to connect Robot!")
                continue
            break

    def TCP_Close(self):
        # self.print_lock.release()
        self.s.close()

    def convert_msg(self, msg):
        try:
            data = msg.split(",")
            if len(data) == 6:
                floats_list = [float(x) for x in data]
                # print("Float list: ", floats_list)
                robot_pose = [round(elem, 4) for elem in floats_list]
                return robot_pose
        except:
            return False


    def modify_pose(self, pose):
        self.pose = pose

    def send_command(self, command):
        # self.s.settimeout(5)
        try:
            self.s.sendall(bytes(command, "utf8"))
            # print("Command sended: ", command)
            self.retry_counter = 0
            self.sended_flag = True
            self.command_executed = False
        except socket.error as e:
            print("Send error: ", e)
            if self.retry_counter <=5:
                self.retry_counter += 1
                print("Retry sending command time: ", self.retry_counter)
                self.send_command(command)
            else:
                self.retry_counter = 0
                sys.exit("Fail to send command to Robot!")

    def get_response(self):
        while True:
            try:
                msg = self.s.recv(1024)
            except socket.timeout as e:
                err = e.args[0]
                if err == 'timed out':
                    # sleep(1)
                    if self.sended_flag:
                        if self.timeout_counter <= 3:
                            print ('Wait for comfirm ends in: ', 3 - self.timeout_counter)
                            self.timeout_counter += 1
                        else:
                            print("Fail to comfirm")
                            self.timeout_counter = 0
                            self.sended_flag = False
                    continue
                else:
                    print (err)
            except socket.error as e:
                err = e.args[0]
                # Something else happened, handle error, exit, etc.
                # if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                print ("Error: ", e)
                sys.exit(1)
            else:
                if len(msg) == 0:

                    print ('Robot disconnected')
                    self.exit_flag = True
                    sys.exit(0)
                else:
                    utf8_msg = msg.decode("utf8")
                    print('Robot send msg: ', utf8_msg)

                    if self.sended_flag:
                        if self.command_name == "O" and utf8_msg == "Opened":
                            print ("Gripper is opened")
                            self.command_executed = True                          
                            self.timeout_counter = 0
                            self.sended_flag = False
                        elif self.command_name == "C" and utf8_msg == "Miss":
                            print ("Grasp miss")
                            self.sended_flag = False
                            self.timeout_counter = 0
                            # self.command_executed = True
                        elif self.command_name == "C" and utf8_msg == "Grapped":
                            print ("Grasped")
                            self.command_executed = True
                            # sleep(1)
                            self.timeout_counter = 0
                            self.sended_flag = False
                        elif self.command_name == "M":
                            robot_pose = self.convert_msg(utf8_msg)
                            if not robot_pose == False:
                                newpose = [round(elem, 4) for elem in self.pose]
                                if robot_pose == newpose:
                                    print ("Move completed")
                                    self.command_executed = True
                                    self.timeout_counter = 0
                                    self.sended_flag = False
                                else:
                                    print("Move failed")


    def goto_pose(self, pose):
        # a = list(map(int, 
        # input("\nEnter the numbers : ").strip().split()))[:6]
        if self.sended_flag:
            print("Previous command hasn't been comfirmed!", self.command_name)
        else:
            self.pose = pose
            strpose = re.sub("\[|\]", "", str(pose))
            self.command_name = "M"
            self.send_command("M, " + strpose)
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
        #return self.get_response()
        
    def move_home(self):
        # a = list(map(int, 
        # input("\nEnter the numbers : ").strip().split()))[:6]
        if self.sended_flag:
            print("Previous command hasn't been comfirmed!", self.command_name)
        else:
            self.pose = [138, -244, 413, 179, -0.9, 179]
            strpose = re.sub("\[|\]", "", str(self.pose))
            self.command_name = "M"
            self.send_command("M, " + strpose)
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
        #return self.get_response()
            
    def open_gripper(self):
        if self.sended_flag:
            print("Previous command hasn't been comfirmed!", self.command_name)
        else:
            self.command_name = "O"
            self.send_command("O")
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False
        
    def grasp(self):
        if self.sended_flag:
            print("Previous command hasn't been comfirmed!", self.command_name)
        else:
            self.command_name = "C"
            self.send_command("C")
            while self.sended_flag:
                pass
            if self.command_executed:
                return True
        return False

    # thread function
    def threaded(self):
        self.get_response()
        self.print_lock.release()
        print("End listenner thread")
        sys.exit(0)
            
    def run_listenner(self):
        # lock acquired by client
        self.print_lock.acquire()
        # Start a new thread and return its identifier
        print("Start new thread to listen from Robot")
        start_new_thread(self.threaded, ())