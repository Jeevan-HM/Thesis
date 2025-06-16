"""
This code is the PC Client
normal to compliant to recovery 
recovery to compliant


"""
import numpy as np
import zmq
import pickle
import zlib
from time import time, sleep
import threading
import socket
import struct 
import re
import os
import datetime

class pc_client(object):
    """docstring for pc_client"""
    def __init__(self):
        """ Select use mocap or not"""

        # creating an empty list
        #self.NArs = [1,2,3,4]
        
        self.NArs = [4, 7, 8]

        # check is user is ready
        self.ready = input("If Ready Enter 'y': ")
         
        if self.ready == "y":

            self.flag_use_mocap=1
            self.flag_control_mode=1# 0: baseline smc; 
                                    # 1: smc+ilc;
                                    # 2: smc+spo;
            self.flag_reset = 1
            self.trial_start_reset = 1

            """ Initiate ZMQ communication"""
            context = zmq.Context()

            self.client_sockets = []
            self.client_addresses = []

            self.pc_addr = '10.203.49.197'

            #Arduino Ports
            self.ports = [10001,10002,10003,10004,10005,10006,10007,10008]


    ############################Arduino implementation######################
            for i in range(0,len(self.NArs)):
                i = self.NArs[i]

                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.pc_addr, self.ports[i-1]))
                self.server_socket.listen(1)
                print("Waiting for Arduino Connection...")
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"Connected to Arduino at {self.client_address}")
                self.client_sockets.append(self.client_socket)


    ########################Arduino edit ends here#########################

            #print(self.client_sockets)
            self.socket1 = context.socket(zmq.PUB)##PUb to Record
            self.socket1.setsockopt(zmq.CONFLATE,True)
            self.socket1.bind("tcp://127.0.0.1:5555")
            ##print("here")
            
            self.socket2=context.socket(zmq.SUB) ### sub mocap data
            self.socket2.setsockopt_string(zmq.SUBSCRIBE,'', encoding='utf-8')
            # self.socket2.
            self.socket2.setsockopt(zmq.CONFLATE,True)

            if self.flag_use_mocap == True:
                self.socket2.connect("tcp://127.0.0.1:3885")
                print ("Connected to mocap")

            print ("Connected to Low")

            """ Format recording """
            self.array3setswithrotation=np.array([0.]*21)# base(x y z qw qx qy qz) top(x1 y1 z1 qw1 qx1 qy1 qz1)
            self.pd_array_1=np.array([0.]*len(self.NArs)) #pd1
            #self.pm_array_1=np.array([0.]*len(self.NArs)) #pm1  (psi)
            self.pm_array_1= [None] * len(self.NArs)
            #self.pd_pm_array_2 = self.pd_pm_array_1
            #self.filt_array_wireEnco = np.array([0.]*4)
            # assmble all to recording
            self.arr_comb_record=np.concatenate((self.pd_array_1, self.pm_array_1, self.array3setswithrotation), axis=None)
            #self.arr_comb_record=np.concatenate((self.pd_pm_array_1, self.pd_pm_array_2, self.filt_array_wireEnco, self.array3setswithrotation,self.pd_pm_array_add), axis=None)
            

            """ Thearding Setup """
            self.th1_flag=True
            self.th2_flag= True
            self.th3_flag = True

            self.run_event=threading.Event()
            self.run_event.set()
            self.th1= threading.Thread(name='raspi_client',target=self.th_pd_gen)
            self.th2= threading.Thread(name='mocap',target=self.th_data_exchange)
            self.th3= threading.Thread(name='pub2rec',target=self.th_data_exchange_high)
            #self.th4= threading.Thread(name = 'record_data', target=self.record_data)

            # """ Common variable"""
            # self.t0_on_glob = time()

            # """Input signal selection"""
            self.positionProfile_flag=2#  0: sum of sine waves 1: single sine wave, 2: step
            self.trailDuriation=20.0#sec

    def th_pd_gen(self):
        print("th_Pd_G")
        try:
            if self.flag_reset==1:

                # seg_5 = 0 #r1
                # seg_6 = 0 #sens
                # seg_7 = 0 #r2
                # seg_8 = 0
                # seg_4 = 2

                seg_1 = 2
                # seg_2 = 0
                seg_3 = 0
                seg_4 = 0

                array = [seg_1, seg_3, seg_4]

                # array = [seg_7,seg_8]

                self.t0_on_trial = time()
                self.pres_single_step_response((array),10)


                self.flag_reset=0
            self.t0_on_glob = time()
            #print(time()-self.t0_on_glob)
            while (time()-self.t0_on_glob < self.trailDuriation):
                #print("here")
                try:
                    #print("heheheh")
                    #if self.flag_use_mocap == True:
                        #self.array3setswithrotation=self.recv_cpp_socket2()
                    up_rate = 1.0 # psi/s
                    down_rate = 1.0 # psi/s
                    lower_bound = 0.0 #psi
                    upper_bound = 10.0 #psi
                    #print("hehehe")
                    for i in range(6):
                        if self.trial_start_reset == 1:
                            self.t0_on_trial = time()
                            self.trial_start_reset = 0
                        #self.pres_single_step_response(np.array([0.0]*len(self.NArs)),10)
                        #self.t0_on_trial = time()
                        #self.pres_single_step_response(np.array([5.0]*len(self.NArs)),10)
                        self.pres_single_ramp_response(up_rate,down_rate,upper_bound,lower_bound)
                        self.trial_start_reset = 1
                    
  
                except KeyboardInterrupt:
                    break
                    print("E-stop")
                    self.th1_flag=False
                    self.th2_flag=False
            if self.flag_reset==0:
                self.t0_on_trial = time()
                self.pres_single_step_response(np.array([0.]*len(self.NArs)),10)
                self.flag_reset=1
            self.th1_flag=False
            self.th2_flag=False
            print ("Done")
            exit()
        except KeyboardInterrupt:
            self.th1_flag=False
            self.th2_flag=False
            print ("Press Ctrl+C to Stop")


    def th_data_exchange(self):# thread config of read data from mocap and send packed msg to record file.
        print("th_data_ex")
        # print("Run State: ", self.run_event.is_set())
        # print("th2_flag: ", self.th2_flag)
        # print("Mocap Flag: ", self.flag_use_mocap)
        while self.run_event.is_set() and self.th2_flag:
            try:
                if self.flag_use_mocap == True:
                    self.array3setswithrotation = self.recv_cpp_socket2()  #ADD PUBSUB Pm Pd
                    # print(self.array3setswithrotation)
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.arr_comb_record)

            except KeyboardInterrupt:
                print(Exception)
                break
                self.th1_flag=False
                self.th2_flag=False
                exit()

    def experiment_number(self):
        experiment_date = datetime.datetime.today().strftime('%B-%d')
        experiment_dir = f'experiments/{experiment_date}'
        
        # Ensure the directory exists
        os.makedirs(experiment_dir, exist_ok=True)

        # Get the next experiment number
        existing_numbers = [int(re.search(r'Test_(\d+)\.txt', f).group(1)) 
                            for f in os.listdir(experiment_dir) if re.match(r'Test_\d+\.txt', f)]
        
        # Construct and return the dynamic filename
        experiment_number = max(existing_numbers, default=0) + 1
        return f'experiments/{experiment_date}/Test_{experiment_number}.txt'

    def th_data_exchange_high(self):# thread config of read data from mocap and send packed msg to record file.
        print("th_data_exHIGH")
        # file_name = 'experiments/June5th/Test_2.txt'
        file_name = self.experiment_number()
        print("Logging to: ", file_name)
        start_time = time()
        received_count = 0
        self.comm_rate = []


        with open(file_name,'w+') as data_file:
            t0 = time()
            while self.run_event.is_set() and self.th3_flag:
                try:
                    #timestamp = np.array(round((time()-t0),6))
                    self.arr_comb_record=np.concatenate((self.pd_array_1, self.pm_array_1, self.array3setswithrotation), axis=None)
                    

                    #print(self.arr_comb_record)
                    msg=self.arr_comb_record
                    #print(msg)
                    
                    lines=""
                        
                    #Set Structure
                    lines=str(round((time()-t0),6))+","+np.array2string(msg,separator=',').replace('[','').replace(']','').replace('\n','')+"\n"
                    data_file.write(lines)
                    data_file.flush()

                    

                    received_count+=1


                    current_time = time()
                    if current_time - start_time >= 10:
                        communication_rate = received_count / (current_time - start_time)
                        self.comm_rate.append(communication_rate)
                        #print(f"Communication Rate: {communication_rate:.2f} Hz")
                        start_time = current_time
                        received_count = 0

                    if self.flag_reset==0:
                        self.send_zipped_socket1(self.arr_comb_record)

                except KeyboardInterrupt:
                    break
                    exit()


    def pres_single_ramp_response(self,up_rate,down_rate,upper_bound,lower_bound):
        #print("ramping")
        t = time() - self.t0_on_trial # range from 0
        total = (upper_bound - lower_bound)/up_rate + (upper_bound - lower_bound)/down_rate
        while (self.th1_flag and self.th2_flag and (t <= total)):

            try:
                t = time() - self.t0_on_trial # range from 0
                if (t <= (upper_bound - lower_bound)/up_rate):
                    for i in range(len(self.NArs)):
                        self.pd_array_1[i] = lower_bound + up_rate*t # comment out the code from this line up until line 250 for sine wave

                if (((upper_bound - lower_bound)/up_rate <t ) and (t <= total)):
                    for i in range(len(self.NArs)):
                        self.pd_array_1[i] = upper_bound - down_rate*(t - (upper_bound - lower_bound)/up_rate)
                
                for i in range(len(self.NArs)):        
                    if i == 2 or i == 3 :
                        # self.pm_array_1[i] = self.ard_socket(3,self.client_sockets[i])
                        self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])

                    else:
                        self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])

                
                # for i in range(len(self.NArs)):        
                #     if i == col:
                #         self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
                #     else:
                #         self.pm_array_1[i] = self.ard_socket(0,self.client_sockets[i])
                
                #self.pm_array_1 = self.ard_socket(self.pd_array_1[i],self.client_sockets[3])
                #print(self.pm_array_1)

            except KeyboardInterrupt:
                break
                self.th1_flag = 0
                self.th2_flag = 0
                

    def pres_single_step_response(self,pd_array,step_time):
        #print("stepping")
        t = time() - self.t0_on_trial # range from 0
        while (self.th1_flag and self.th2_flag and (t <= step_time)):
            try:
                t = time() - self.t0_on_trial # range from 0
                for i in range(len(self.NArs)):
                    self.pd_array_1[i] = pd_array[i]
                    self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
                #print(self.pm_array_1)
            except KeyboardInterrupt:
                break
                self.th1_flag = 0
                self.th2_flag = 0
                
###############Arduino Sockets###########################
    
    def ard_socket(self, pd_array, client, flags=0):
        packed_data = struct.pack('f',pd_array)
        client.send(packed_data)
        received_data = b''
        while len(received_data) < 8:
            chunk = client.recv(8)
            if not chunk:
                print("Connection to Arduino closed")
                break
            received_data += chunk

        if len(received_data) < 8:
            print("Data from Arduino is messed")
            return None  # Data is too short, skip processing

        # Unpack the received float
        received_float = struct.unpack('>4h', received_data)
        data = list(received_float)

        for i in range(0,len(data)):
            pressure_volt_1 = received_float[i] * (12.288 / 65536.0)
            data[i] = round((((30.0 - 0.0) * (pressure_volt_1 - (0.1 * 5.0))) / (0.8 * 5.0)),4)

        return data

    def send_zipped_socket1(self, obj, flags=0, protocol=-1):
        """pack and compress an object with pickle and zlib."""
        pobj = pickle.dumps(obj, protocol)
        zobj = zlib.compress(pobj)
        self.socket1.send(zobj, flags=flags)


    def recv_zipped_socket2(self,flags=0):
        """reconstruct a Python object sent with zipped_pickle"""
        zobj = self.socket2.recv(flags)
        pobj = zlib.decompress(zobj)
        return pickle.loads(pobj)


    def recv_cpp_socket2(self):
        strMsg =self.socket2.recv()
        floatArray=np.fromstring(strMsg.decode("utf-8"),dtype = float, sep= ',')
        # print("here")
        # print(floatArray)
        # return self.array3setswithrotation
        return floatArray
        # floatArray=np.fromstring(strMsg)
        # return np.fromstring(strMsg, dtype=float, sep=',')