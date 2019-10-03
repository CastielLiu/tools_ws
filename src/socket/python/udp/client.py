#!/usr/bin/python

# -*- coding: UTF-8 -*-

import os
from socket import *
host = "192.168.0.194" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.connect((host,port))

string = ""
while True:
	data = raw_input("Enter message to send or type 'exit': ")
	#UDPSock.sendto(data, addr)
	UDPSock.send(data)
	#print UDPSock.recv(1024)
	if data == "exit":
		break
UDPSock.close()
os._exit(0)
