#coding:utf-8

import sys
import cv2
import imageio
import os

#不同文件夹的图像汇总在一起做gif时,文件编号不能重复，且应按照顺序递增
#使用该工具进行文件编号重新排布
def main(argv):
	#print(argv)
	if(len(argv) < 6):
		print("require 5 arguments but %d imput!" %(len(argv)-1))
		print("please input images [path],[suffix],[startSeq],[endSeq],[addNum]")
		exit()
	path = argv[1]
	if(path[-1] == "/"):
		path = path[0:-1]
	suffix = argv[2]
	startSeq = int(argv[3])
	endSeq = int(argv[4])
	addNum = int(argv[5])
	if(addNum < endSeq-startSeq and addNum > startSeq-endSeq):
		print("The addNum may cause dst_name same with src_name!")
		return 
	
	#先判断安全性(未处理的原始文件是否可能被覆盖)
	for i in range(startSeq, endSeq+1):
		src_name = path + "/" +str(i) +"." + suffix
		if(os.path.exists(src_name)):
			dst_name = path + "/" +str(i+addNum) +"." + suffix
			if(os.path.exists(dst_name)):
				print("%s -> %s" %(src_name, dst_name))
				print("dst_name %s is exists, operation canceled!" %dst_name)
				return
	#在进行修改
	for i in range(startSeq, endSeq+1):
		src_name = path + "/" +str(i) +"." + suffix
		if(os.path.exists(src_name)):
			dst_name = path + "/" +str(i+addNum) +"." + suffix
			os.rename(src_name, dst_name)

if __name__ == "__main__":
	main(sys.argv)
