import socket
import keyboard
import re
import errno
from time import sleep
import sys
# import thread module
from _thread import *
import threading


class Control_Dobot_TCP (object):
    pose = [-68, -677, 258, 179, -2, 179]
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
                print('Connected to %s port %s' % (address[0], address[1]))
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
                print("Float list: ", floats_list)
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
            print("Command sended: ", command)
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
                #print("hello")
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
                    print ("Timeout: ", err)
                    # sys.exit(1)
                    #return False
            except socket.error as e:
                err = e.args[0]
                # Something else happened, handle error, exit, etc.
                # if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                print ("Err: ", err)
                print ("Error: ", e)
                #self.print_lock.release()
                sys.exit(1)
                #return False
            else:
                if len(msg) == 0:
                    #self.print_lock.release()
                    print ('Shutdown on server end')
                    self.exit_flag = True
                    #sys.exit(0)
                    self.TCP_Close()
                    break
                else:
                    # got a message do something :)
                    utf8_msg = msg.decode("utf8")
                    print('Robot: ', utf8_msg)

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
                            print ("Pose match")
                            self.command_executed = True
                            self.timeout_counter = 0
                            self.sended_flag = False
                            # if not robot_pose == False:
                            #     newpose = [round(elem, 4) for elem in self.pose]
                            #     if robot_pose == newpose:
                            #         print ("Pose match")
                            #         self.command_executed = True
                            #         self.timeout_counter = 0
                            #         self.sended_flag = False
                            #     else:
                            #         print("Pose not match")
                            # else:
                            #     print("Unknow comfirm!")
                        else:
                            print("Unknow comfirm!")

                            
                        #return True
                #return False

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
                print("Open finish")
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
                print("Grasped finish")
                return True
        return False

    # thread function
    def threaded(self):
        self.get_response()
        self.print_lock.release()
        print("End thread")
        sys.exit(0)
        # self.TCP_Close()
            
    def create_listen(self):
        # lock acquired by client
        self.print_lock.acquire()
        # Start a new thread and return its identifier
        print("Start thread")
        start_new_thread(self.threaded, ())

# def main():
#     print("Control Dobot Start")
#     HOST = '127.0.0.1'  
#     PORT = 7001
#     address = (HOST, PORT)
#     dobot_control = Control_Dobot_TCP(address)
#     pose = [0, -500, 300, 179, -2, 179]
#     dobot_control.create_listen()
#     while True:
#         pass
        # if keyboard.read_key() == "p":
        #     a = list(map(float, 
        #     input("\nEnter the numbers : ").strip().split()))[:6]
        #     #print("Send Move Command")
        #     if len(a) == 6:
        #         print("Move: ", dobot_control.move_robot(a))
        #     else:
        #         print("Move: ", dobot_control.move_robot(pose))

        # if keyboard.read_key() == "o":
        #     #print("Send Open Command")
        #     print ("Open gripper: " ,dobot_control.open_gripper())
        
        # if keyboard.read_key() == "c":
        #    # print("Send Close Command")
        #     print ("Graps: " ,dobot_control.grasp())
        
        # if keyboard.read_key() == "q":
        #     print("Client disconnect")
        #     #dobot_control.TCP_Close()
        #     break   

# if __name__ == "__main__":
#     main()