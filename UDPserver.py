# Import the necessary socket python module
import socket;
# Define the IP address as well as the port number
UDP_IP_ADDRESS = '192.168.56.102'
UDP_PORT_NO = 8888

# Declare the serverSocket which will be listening
# for UDP messages
serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the IP address and port number to serverSock
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))
while True:
  data, addr = serverSock.recvfrom(512)
  print ("Received {} from {}.".format(data, addr))
  message = 'ping'
  serverSock.sendto(message.encode('utf-8'), addr)
