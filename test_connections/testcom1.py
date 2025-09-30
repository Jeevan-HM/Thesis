# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 10:25:19 2023

@author: Jahnav Rokalaboina
"""

import socket
import struct
import time
import csv

# Set the PC's IP address and port
pc_ip = '10.203.50.9'  # Replace with the PC's actual IP address "tcp://10.203.50.19:4444"
pc_port = 10003

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((pc_ip, pc_port))
server_socket.listen(1)

start_time = time.time()
sent_count = 0
received_count = 0
array = [0,1,3,1,5,2,4,2,5,1,0,2,4,3,0,1,3,0,5,3,0,4,1,0,3,2,5,4,1]
#for i in range(30):
 #   array.append(random.randint(0,5))


csv_file='inputs.csv'
with open(csv_file,mode='w',newline='') as file:
    writer = csv.writer(file)
    writer.writerow(array)
k=0
increasing=True

while True:
    print("Waiting for Arduino connection...")
    client_socket, client_address = server_socket.accept()
    print(f"Connected to Arduino at {client_address}")

    while True:

        sen_data = 10.0 #*(array[k]) #round(random.uniform(0,5),2) 
        packed_data = struct.pack('f',sen_data)
        client_socket.send(packed_data)
        sent_count+=1
        # Receive sensor data from Arduino (float type)
        # Receive 4 bytes for the float
        received_data = b''
        while len(received_data) < 12:
            chunk = client_socket.recv(12 - len(received_data))
            if not chunk:
                print("Connection to Arduino closed")
                break
            received_data += chunk

        if len(received_data) < 12:
            continue  # Data is too short, skip processing

        # Unpack the received float
        received_float = struct.unpack('>6h', received_data)
        received_count+=1
        print(f"Received data: {received_float}")

        # Calculate and display the communication rate every 2 seconds
        current_time = time.time()
        if current_time - start_time >= 2:
            k=k+1
            communication_rate = received_count / (current_time - start_time)
            print(f"Communication Rate: {communication_rate:.2f} Hz")
            print(f"Sent Count: {sent_count}")
            print(f"Received Count: {received_count}")
            start_time = current_time
            sent_count = 0
            received_count = 0

        # You can send float data to Arduino using client_socket.send() if needed
        
    l = 0
    while l < 13:
        l += 1
        if l in self.NArs:
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.server_socket.bind((self.pc_addr, self.ports[l-1]))
                self.server_socket.listen(1)
                print("Waiting for Arduino Connection...")
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"Connected to Arduino at {self.client_address}")
                self.client_sockets.append(self.client_socket)

