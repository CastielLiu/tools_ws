import socket
host='localhost'
port=12344
addr=(host,port)
tcpCliSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

tcpCliSock.connect(addr)
while True:
	msg = input('please input:')
	if not msg:
		break;
	tcpCliSock.send(msg.encode())
	data=tcpCliSock.recv(2048)
	if not data:
		break;
	print(data.decode())

