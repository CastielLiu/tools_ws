#coding=UTF-8

import sys
import cv2
import imageio
import datetime
import os
sys.path.append("lib/")
from image_mask import imageAddWeighted

def handle(path,startSeq,endSeq,suffix,duration,resolution,scale=None):
	
	#首先按照seq获取所有可用文件名
	imageNames = []
	for i in range(startSeq,endSeq+1):
		imgName = path + "/" + str(i) + "." + suffix
		if(os.path.exists(imgName)):
			imageNames.append(imgName)
	
	totalFrameCnt = len(imageNames)						#图像总数
	needFrameCnt = int(totalFrameCnt*1.0/resolution)	#根据取值精度求需要的图像数量
	print("total: %d\t need: %d" %(totalFrameCnt, needFrameCnt))
	if(totalFrameCnt==0 or needFrameCnt==0):
		return
	
	imageBuf = []
	
	#添加指定位置加权添加mask,　如需此功能，需在图像同级目录放置mask.jpg
	mask = None
	mask_file = path+"/mask.jpg"
	if(os.path.exists(mask_file)):
		mask = cv2.imread(mask_file)
	mask_loc = (580,10)   #mask位置
	mask_ratio = (0,1)    #原图与mask各占比例
	
	#利用需要的图像数量插值查找图像并添加进imageBuf
	for i in range(needFrameCnt+1):
		j = int(i*1.0/needFrameCnt*totalFrameCnt)

		if(j >= totalFrameCnt):
			j = j-1
				
		img = cv2.imread(imageNames[j])
		if(img is None):
			print("read %s failed" %imageNames[j])
			continue
		if(mask is not None):
			imageAddWeighted(img, mask, mask_loc, mask_ratio)
		
		print("append image: %s" %imageNames[j])
		
		if(scale):
			img =cv2.resize(img,(0,0),fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
		
		imageBuf.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
	
	now = datetime.datetime.now()
	outName = ("%s%s%s-%s%s%s.gif" %(now.year,now.month,now.day,now.hour,now.minute,now.second))
	if(path[-1] == "/"):
		outName = path[0:-1] + "_" + outName
	else:
		outName = path + "_" + outName
	imgCnt = len(imageBuf)

	duration = duration*1.0/imgCnt

	print("saving %s ..." %outName)
	gif=imageio.mimsave(outName,imageBuf,'GIF',duration=duration)

	print("%s contains %d frame images, total time is %.2fs" %(outName,imgCnt,duration*imgCnt))

#path startSeq,endSeq,suffix,duration 
def main(argv):
	if(len(argv) < 6):
		print("please input images [path],[startSeq],[endSeq],[suffix],[duration] (resolution, scale)")
		exit()
	path = argv[1]
	startSeq = int(argv[2])
	endSeq = int(argv[3])
	suffix = argv[4]
	duration = float(argv[5])
	if(len(argv) > 6):
		resolution = int(argv[6])
		if(resolution < 1.0):
			resolution = 1.0
	else:
		resolution = 1.0
	
	if(len(argv) > 7):
		scale = float(argv[7])
	else:
		scale = None
	
	handle(path,startSeq,endSeq,suffix,duration,resolution,scale)
	
if __name__ == "__main__":
	main(sys.argv)
