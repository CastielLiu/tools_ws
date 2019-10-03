import socket

host=''
port=12344
addr=(host,port)
tcpSerSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
tcpSerSock.bind(addr)
tcpSerSock.listen(5)

while True:
	print('waiting for client...')
	tcpCliSock,client_addr = tcpSerSock.accept() 
	print('connect ok addr:',client_addr)
	while True:
		data=tcpCliSock.recv(2048)
		if not data:
			break
		print(data.decode())
		msg=input('please input:')
		if not msg:
			break;
		tcpCliSock.send(msg.encode())
	tcpCliSock.close()
tcpSerSock.close()

