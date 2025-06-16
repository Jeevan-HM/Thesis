

import numpy as np
import zmq
import pickle
import zlib
from time import time, sleep
import threading
import socket
import struct 


context = zmq.Context()
socket2=context.socket(zmq.SUB) ### sub mocap data
socket2.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
        # self.socket2.
socket2.setsockopt(zmq.CONFLATE,True)

socket2.connect("tcp://127.0.0.1:3885")
print ("Connected to mocap")

while True:
        print("here")
        strMsg =socket2.recv()
        print("heere")
        floatArray=np.fromstring(strMsg.decode("utf-8"),dtype = float, sep= ',')
        # print("here")
        # print(floatArray)
        # return self.array3setswithrotation
        print(strMsg)
        # floatArray=np.fromstring(strMsg)
        # return np.fromstring(strMsg, dtype=float, sep=',')