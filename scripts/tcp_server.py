import socket
import re
import time
HOST = '127.0.0.1'  
# PORT = 8000

import init_conf

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, init_conf.PORT))
s.listen(2)

print("Start server")
while True:
    client, addr = s.accept()

    print('Connected by', addr)
    while True:
        data = client.recv(1024)
        str_data = data.decode("utf8")
        if str_data == "quit":
            break
            """if not data:
                break
            """
        print("Client: " + str_data)
        time.sleep(1)
        if str_data == "O":
            client.sendall(bytes("Opened", "utf8"))
        elif str_data == "C":
            client.sendall(bytes("Grapped", "utf8"))  
        else:
            newlist = list(str_data.split(","))
            newlist.pop(0)
            strpose = re.sub("\[|\]", "", str(newlist))
            client.sendall(bytes(str(strpose), "utf8"))           

        # msg = input("Server: ")

        
    client.close()
    break
s.close()

