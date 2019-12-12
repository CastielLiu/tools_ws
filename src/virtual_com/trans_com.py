#! /usr/bin/env python
#coding=utf-8

"""
串口串联， 将虚拟串口串联在两交互设备中间，并捕获双方消息
暂未实现！！！ 测试出现问题
"""
#coding=utf-8
from __future__ import print_function
import serial
import pty
import os
import sys
import select
from time import sleep
import binascii
import threading
import time


def str_to_hex(s):
	#return ' '.join([hex(ord(c)).replace('0x', '') for c in s])
	return ' '.join(['%02X' %(ord(c)) for c in s])
	
def mkpty():
	master, slave = pty.openpty()
	slave_name = os.ttyname(slave)
	return master, slave_name

thread_flag = True

def main(argv):
	if(len(argv)>2 and argv[1] == "hex"):
		is_hex = True
	else:
		is_hex = False
		
	master1,slave_name1 = mkpty()
	print("Pseudo serial port: %s" %slave_name1)
	
	#pseudo_ser = serial.Serial(slave_name1,115200,timeout=0.5)
	ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.5)
	
	thr1 = threading.Thread(target=thread1,args=(master1, ser),name='t1')
	thr2 = threading.Thread(target=thread2,args=(master1, ser),name='t2')
	thr1.start()
	thr2.start()
	
	try:
		while True:
			time.sleep(10000)
	except KeyboardInterrupt:
		global thread_flag
		thread_flag = False
#	
#	thr1.stop()
#	thr2.stop()

mutex = threading.Lock()

def thread1(pseudo_ser, ser):
	while thread_flag:
		data = os.read(pseudo_ser,128)
		ser.write(data)
		mutex.acquire()
		print("to_lidar: %s" %str_to_hex(data))
		sys.stdout.flush()
		mutex.release()

def thread2(pseudo_ser, ser):
	while thread_flag:
		data = ser.read(128)
		os.write(pseudo_ser, data)
		if(len(data) == 0):
			continue
		mutex.acquire()
		print("from_lidar: %s" %str_to_hex(data))
		sys.stdout.flush()
		mutex.release()
		

if __name__ == "__main__":
	try:
		main(sys.argv)
	except KeyboardInterrupt:
		print("")
		pass
		
		
		
