import socket
import struct
import time
import numpy as np

pc_addr = '10.203.49.197'
ports = []
NArs = []
client_sockets = []
checklist = []

n = int(input("Enter number of towers : "))
         
for i in range(0, n):
	ele = int(input())
	NArs.append(ele)
#print(NArs)

#Arduino Ports
for i in range (1,13):
	ports.append(10000 + i)
#print(ports)

l = 0
k=0
start_time = time.time()
sent_count = 0
received_count = 0

#Arduino Connection       	
while l < 13:
	l += 1
	if l in NArs:
        	server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        	server_socket.bind((pc_addr, ports[l-1]))
        	server_socket.listen(1)
        	print("Waiting for Arduino Connection...")
        	client_socket, client_address = server_socket.accept()
        	print(f"Connected to Arduino at {client_address}")
        	client_sockets.append(client_socket)
            
#print(client_sockets)
pm_array=np.array([0.]*len(NArs))

set = input("If ready enter 'y': ")

while set == "y":
	for i in range(len(NArs)):
		sen_data = 10.0 #*(array[k]) #round(random.uniform(0,5),2) 
		#print("Confirmed Sen_data")
		time.sleep(0.001)
		packed_data = struct.pack('f',sen_data)
		#print("Packed Data")
		client_sockets[i].send(packed_data)
		#print("Sent data to client socket")
		#sent_count+=1
		# Receive sensor data from Arduino (int type)
		# Receive 4 bytes for the int
		received_data = b''
		#print("Received Data: ",received_data)
		while len(received_data) < 12:
			chunk = client_sockets[i].recv(12)
			
			#print(12 - len(received_data))
			if not chunk:
				print("Connection to Arduino closed")
				break
			received_data += chunk
			#print(getsizeof(received_data))
		#print(len(received_data))
		if len(received_data) < 12:
		    continue  # Data is too short, skip processing
		    print("data too short")
		# Unpack the received float
		#time.sleep(0.001)
		received_float = struct.unpack('>6h', received_data)
		received_count+=1
		 
		data = received_float[0]
		pressure_volt_1 = data * (5.0 / 1023.0)
		pm_array[i] = round(((60.0 - 0.0) * (pressure_volt_1 - (0.1 * 5.0))) / (0.8 * 5.0),2)
		#pm_array[i] = received_float[3]

		#print("Ard " + str(i+1))
		print(pm_array)

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