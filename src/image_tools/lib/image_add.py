#coding:utf-8

import sys
import cv2
import imageio
import os

#位置矫正,当图像添加的位置超出图像范围时进行位置修正
def locationCorrect(w1,h1,w2,h2,loc):
	outLoc = list(loc)
	if(loc[0] < 0):
		outLoc[0] = 0
		
	if(loc[1] < 0):
		outLoc[1] = 0
	
	if(loc[0]+w2 >w1 or loc[1]+h2 >h1):
		dis2down = h1 - loc[1]
		dis2right = w1 - loc[0]
		
		#下右均超出范围
		if(dis2down < h2 and dis2right < w2):
			outLoc[0] = w1 - w2
			outLoc[1] = h1 - h2
		#下方超出范围
		elif(dis2down < h2):
			outLoc[1] = h1 - h2
		#右方超出范围
		elif(dis2right < w2):
			outLoc[0] = w1 - w2
		
	return tuple(outLoc)

#图片叠加函数，在image1上添加image2
#image1的尺寸必须大于image2
#参数loc为添加位置,默认为0,右上角; 1左上角; 2左下角; 3右下角
def imageStdAdd(image1, image2, loc=0):
	w1 = image1.shape[1]
	h1 = image1.shape[0]

	w2 = image2.shape[1]
	h2 = image2.shape[0]

	if(w2 > w1 or h2 > h1):
		print("The image2 is bigger than image1!")
		return None

	if(loc == 0):
		rowBegin = 0
		colBegin = w1-w2
	elif(loc == 1):
		rowBegin = 0
		colBegin = 0
	elif(loc == 2):
		rowBegin = h1-h2
		colBegin = 0
	else: #(loc == 3)
		rowBegin = h1-h2
		colBegin = w1-w2
	
	for row in range(image2.shape[0]):
		for col in range(image2.shape[1]):
			#for channel in range(image2.shape[2]):
			image1[row+rowBegin, col+colBegin] = image2[row, col]
	return image1

#图片叠加函数，在image1上添加image2
#image1的尺寸必须大于image2
#loc: 添加位置,默认为(0,0)
#filter_: 过滤像素值, 默认不过滤
def imageAdd(image1,image2,loc=(0,0),filter_=None):
	w1 = image1.shape[1]
	h1 = image1.shape[0]

	w2 = image2.shape[1]
	h2 = image2.shape[0]
	
	loc = locationCorrect(w1,h1,w2,h2,loc)
	
	rowBegin = loc[1]
	colBegin = loc[0]

	for row in range(image2.shape[0]):
		for col in range(image2.shape[1]):
			#for channel in range(image2.shape[2]):
			if(filter_ and image2[row, col] == filter_):
				continue;
			image1[row+rowBegin, col+colBegin] = image2[row, col]
	return image1
	
	#	scale = 0.4
	#	image2 =cv2.resize(image2,(0,0),fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

#图片加权叠加函数，在image1上添加image2
#image1的尺寸必须大于image2
#loc: 添加位置,默认为(0,0)
#ratio: 像素值比例，透明度,默认为(0,1),不透明
def imageAddWeighted(image1,image2,loc=(0,0),ratio=(0,1)):
	if(ratio==(0,1)):
		return imageAdd(image1,image2,loc)
	if(ratio==(1,0)):
		return image1
	
	w1 = image1.shape[1]
	h1 = image1.shape[0]

	w2 = image2.shape[1]
	h2 = image2.shape[0]
	
	loc = locationCorrect(w1,h1,w2,h2,loc)
	
	rowBegin = loc[1]
	colBegin = loc[0]
	
	k1 = ratio[0]/(ratio[0]+ratio[1])
	k2 = 1 - k1
	for row in range(image2.shape[0]):
		for col in range(image2.shape[1]):
			for channel in range(image2.shape[2]):
				image1[row+rowBegin, col+colBegin, channel] = int(image1[row+rowBegin, col+colBegin, channel]*k1 + \
																  image2[row, col, channel] * k2)
	return image1


def demo(argv):
	image1 = cv2.imread(argv[1])
	image2 = cv2.imread(argv[2])
	image3 = imageAdd(image1,image2, (39,969))
	if(image3 is not None):
		cv2.namedWindow("IMG",0)
		cv2.imshow("IMG",image3)
		cv2.waitKey(0)
	
