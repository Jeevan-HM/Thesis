""" Chach's personal input for ramp signal """ 

import numpy as np 
import zmq
import pickle 
import zlib 
import threading 
import socket 
import struct 
from  time import time

class pc_client(object): # using class(blueprint) to create object 
    def __init__(self): 
        # number of selected arduinos total of 8 
        self.Nars = [1,2,3,4]  
        # number of elements as input
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
            self.ports = []
            for i in range (1,9):
                self.ports.append(10000 + i)

    ############################Arduino implementation######################
            for i in range(1,9):
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
            self.pd_array_1=np.array([0.]*len(self.Nars)) #pd1
            self.pm_array_1=np.array([0.]*len(self.Nars)) #pm1  (psi)
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
        #print("th_Pd_G")
        try:
            if self.flag_reset==1:
                
                #change PSI lvl in each section of the column

                seg_1 = 3
                seg_2 = 0
                seg_3 = 3
                seg_4 = 0
                seg_5 = 0
                seg_6 = 0
                seg_7 = 0
                seg_8 = 0

                array = [seg_1,seg_2,seg_3,seg_4,seg_5,seg_6,seg_7,seg_8] # array for the client socket protocol for PD 

                self.t0_on_trial = time()
                self.pres_single_step_response((array),20)

                # seg_1 = 4
                # seg_2 = 4
                # seg_3 = 4
                # seg_4 = 4
                # seg_5 = 0
                # seg_6 = 0
                # seg_7 = 0
                # seg_8 = 0

                # array = [seg_1,seg_2,seg_3,seg_4,seg_5,seg_6,seg_7,seg_8]

                # self.t0_on_trial = time()
                # self.pres_single_step_response((array),10)

                # seg_1 = 0
                # seg_2 = 0
                # seg_3 = 0
                # seg_4 = 0
                # seg_5 = 0
                # seg_6 = 0
                # seg_7 = 0
                # seg_8 = 0

                # array = [seg_1,seg_2,seg_3,seg_4,seg_5,seg_6,seg_7,seg_8]

                # self.t0_on_trial = time()
                # self.pres_single_step_response((array),10)


                self.flag_reset=0
            self.t0_on_glob = time()
            print(time()-self.t0_on_glob)
            while (time()-self.t0_on_glob < self.trailDuriation):
                #print("here")
                try:
                    #print("heheheh")
                    #if self.flag_use_mocap == True:
                        #self.array3setswithrotation=self.recv_cpp_socket2()
                    up_rate = 1 # psi/s
                    down_rate = 1 # psi/s
                    lower_bound = 0.0 #psi
                    upper_bound = 5.0 #psi # DO NOT GO OVER 7-8 PSI !!!
                    #print("hehehe")
                    for i in range(5):
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
        while self.run_event.is_set() and self.th2_flag:
            try:
                if self.flag_use_mocap == True:
                    self.array3setswithrotation = self.recv_cpp_socket2()  #ADD PUBSUB Pm Pd
                if self.flag_reset==0:
                    self.send_zipped_socket1(self.arr_comb_record)

            except KeyboardInterrupt:
                break
                self.th1_flag=False
                self.th2_flag=False
                exit()

    def th_data_exchange_high(self):# thread config of read data from mocap and send packed msg to record file.
        print("th_data_exHIGH")
        file_name='PR_Computing_task2.txt' # change the text file name every run to prevent overwriting ex: 'data_collect_random2.txt'

        with open(file_name,'w+') as data_file:
            t0 = time()
            while self.run_event.is_set() and self.th3_flag:
                try:
                    self.arr_comb_record=np.concatenate((self.pd_array_1, self.pm_array_1, self.array3setswithrotation), axis=None)
                    #print(self.arr_comb_record)
                    msg=self.arr_comb_record
                    #print(msg)
                    
                    lines=""
                    start_time = time()
                    told=t0
                        
                    #Set Structure
                    lines=str(round((time()-t0),6))+","+np.array2string(msg,separator=',').replace('[','').replace(']','').replace('\n','')+"\n"
                    data_file.write(lines)
                    data_file.flush()

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
                    for i in range(len(self.Nars)):
                        self.pd_array_1[i] = lower_bound + up_rate*t

                if (((upper_bound - lower_bound)/up_rate <t ) and (t <= total)):
                    for i in range(len(self.Nars)):
                        self.pd_array_1[i] = upper_bound - down_rate*(t - (upper_bound - lower_bound)/up_rate)

                for i in range(len(self.Nars)):            
                    self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])

                print(self.pm_array_1)
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
                for i in range(len(self.Nars)):
                    self.pd_array_1[i] = pd_array[i]
                    self.pm_array_1[i] = self.ard_socket(self.pd_array_1[i],self.client_sockets[i])
                #print(self.pm_array_1)
            except KeyboardInterrupt:
                break
                self.th1_flag = 0
                self.th2_flag = 0

######## Arduino Sockets #####

    def ard_socket(self, pd_array, client, flags=0):

        packed_data = struct.pack('f',pd_array)
        client.send(packed_data)
        received_data = b''
        while len(received_data) < 12:
            chunk = client.recv(12)
            if not chunk:
                print("Connection to Arduino closed")
                break
            received_data += chunk

        if len(received_data) < 12:
            print("Data from Arduino is messed")  
            return None  # Data is too short, skip processing

        # Unpack the received float
        received_float = struct.unpack('>6h', received_data)
        data = received_float[0] # changing between arduino analog ports for data collection type 
        
        pressure_volt_1 = data * (5.0 / 1023.0)
        pm = round(((60.0 - 0.0) * (pressure_volt_1 - (0.1 * 5.0))) / (0.8 * 5.0),2)
        w_length = data * 1000 / 1023
        return pm
    
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
