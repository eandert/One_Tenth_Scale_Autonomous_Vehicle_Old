import socket
import time
import sys
import fcntl
import struct
import math
import os
import signal
import subprocess
import psutil
from io import StringIO
import csv

# Start importing cryptographic libraries
import hashlib


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])

#TODO: Protect everything within TLS
    # RECEIVE: pk_RSU, symk_session
    # SEND: pk_CAV
class connectServer:
    def __init___(self):
        self.connectServer()

    #TODO: Send shared secret
    def handshake():
        return None

    def connectServer():
        while True:
            try:
                self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server.bind((get_ip_address(b"wlan0"),0))

                 #TODO: Hardcoded IP addresses. Change so general purpose
                self.server.connect((b"192.168.1.162", 12345))
                break
            except socket.error:
                print ( " Error: Connection to RSU refused, retrying.." )
                time.sleep(1)
            except Exception as e:
                print ( " Error: Cannot find wlan0, retrying..", str(e)  )
                time.sleep(1)

    def messageLocation(self, vehicle_id, timestamp, position, detections):
        send_to_cental = str(vehicle_id) + ':' + timestamp + ':' + position + '\0'
        try:
            server.sendall(send_to_cental.encode('utf-8'))
        except:
            print ( " Error: RSU send error" )

        # Receive data from the server and shut down
        try:
            received = str(server.recv(1024).strip())
        except:
            print ( " Error: RSU recieve error" )
            received = ""
        
        if len(received) > 0:
            if "ACK" not in recieved:
                print ( " Error: Did not recieve ACK from RSU" )
        else:
            print ( "Error: No (ACK) response from RSU" )
            self.server = self.connectServer()


class connectLIDAR:
    def __init__(self, pipeFromC, pipeToC):
        self.pipeFromC = pipeFromC
        self.pipeToC =  pipeToC
        self.lidarTimeout = 1
        self.time = time.time()
        self.killMapdemo()
        self.debug =  False
        self.localization = [0.0,0.0,0.0]
        time.sleep(1)
        try:
            os.mkfifo(pipeFromC)
        except OSError as oe:
            print ( "  Warning: pipeFromC exists, that is cool we will use it" )
        try:
            os.mkfifo(pipeToC)
        except OSError as oe:
            print ( "  Warning: pipeToC exists, that is cool we will use it" )
        self.lidarProc = self.runLIDARCode()
        time.sleep(1)
        lidarTimeout = 0
        self.connectLIDAR()

    def connectLIDAR(self):
        tries = 0
        while True:
            try:
                tries += 1
                # Now start the opening process
                toc=open(self.pipeToC,'w')
                toc.flush()
                toc.write("S")
                toc.close()
                if self.debug:
                    print ( "Wrote pipe" )

                fromc=open(self.pipeFromC,'r')
                str=fromc.read()
                if "A" in str:
                    fromc.close()
                    print ("Sucess, LIDAR started!")
                    return
                else:
                    print (" Error: LIDAR not started.")
                fromc.close()
            except Exception as e:
                print ( " Error: Cannot talk to the LIDAR, retrying..", str(e) )
                # Set tries to 11 so that it reconnects, probably a seg fault
                tries = 11
                time.sleep(1)
            if tries > 10:
                self.killMapdemo()
                time.sleep(1)
                self.lidarProc = self.runLIDARCode()                 
                time.sleep(1)
                tries = 0

    def runLIDARCode(self):
        cmd = "./slamware_sdk_linux-armv7hf-gcc4.8/linux-armv7hf-release/output/mapdemo"
        pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
        return pro

    def killMapdemo(self):
        PROCNAME = "mapdemo"
        for proc in psutil.process_iter():
        # check whether the process name matches
            if proc.name() == PROCNAME:
                proc.kill()

    def checkFromC(self):
        if self.debug:
            print("Opening FIFO...")
        fromc=open(self.pipeFromC,'r')
        self.time = time.time()
        self.datastore = fromc.read()
        if self.debug:
            print('Read: "{0}"'.format(self.datastore))
        fromc.close()

    def parseFromC(self):
        reader = csv.reader(self.datastore.split('\n'), delimiter=',')
        notfirst = False
        lidarpoints = []
        try:
            for idx, row in enumerate(reader):
                if notfirst:
                    if row[0] == "1":
                        angle = float(row[1])
                        distance = float(row[2])
                        newRow = []
                        newRow.append(distance * math.cos(angle))
                        newRow.append(distance * math.sin(angle))
                        lidarpoints.append(newRow)
                else:
                    notfirst = True
                    self.localization[0] = float(row[0])
                    self.localization[1] = float(row[1])
                    self.localization[2] = float(row[2]) 
        except Exception as e:
            print ( " Error: ", str(e) )
        return lidarpoints
