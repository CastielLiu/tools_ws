#coding:utf-8

import sys
import cv2
import imageio
import os

def handle(video,start_time,end_time, resolution):
	if resolution is None:
		resolution = 1
	cap = cv2.VideoCapture(video)
	frame_rate = cap.get(cv2.CAP_PROP_FPS)
	frame_cnt = cap.get(cv2.CAP_PROP_FRAME_COUNT)
	total_time = frame_cnt/frame_rate
	if(end_time > total_time):
		end_time = total_time
	start_frame = start_time*frame_rate
	end_frame = end_time*frame_rate
	total_frame = end_frame - start_frame
	
	cap.set(cv2.CAP_PROP_POS_MSEC,start_time*1000)
	
	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	
#	form = cap.get(cv2.CAP_PROP_POS_FRAMES)
#	print("form", form)
	
	#fourcc = cv2.VideoWriter_fourcc(*'MP42')
	fourcc = cv2.VideoWriter_fourcc(*'XVID')

	prefix = video.split('.')
	outName = prefix[0]+'_out.'+prefix[1]
	print(outName)
	out = cv2.VideoWriter(outName, fourcc, frame_rate, size)
	while True:
		time = cap.get(cv2.CAP_PROP_POS_MSEC)
		if(time >= end_time*1000):
			break
		current_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
		
		ok_frame = current_frame-start_frame
		
		print("%4d/%d" %(ok_frame, total_frame))
		result, img = cap.read()
		if(not result):
			break
		
		if(int(ok_frame)%resolution != 0):
			continue

		out.write(img)
	cap.release()
	out.release()
	print("process ok...")

#图片叠加函数，在image1上添加image2
#image1的尺寸必须大于image2
#参数loc为添加位置,默认为0,右上角
def imageAdd(image1, image2, loc=0):
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
	
	for row in range(image2.shape[0]):
		for col in range(image2.shape[1]):
			#for channel in range(image2.shape[2]):
			image1[row+rowBegin, col+colBegin] = image2[row, col]
	return image1

def fun():
	image1_buf = []
	image2_buf = []
	
	for i in range(200):
		image_name = "7_images/"+str(i)+".jpg"
		if(os.path.exists(image_name)):
			image1_buf.append(cv2.imread(image_name))
		image_name = "5_images/"+str(i)+".jpg"
		if(os.path.exists(image_name)):
			image2_buf.append(cv2.imread(image_name))
	for i in range(200):
		image_name = "6_images/"+str(i)+".jpg"
		if(os.path.exists(image_name)):
			image2_buf.append(cv2.imread(image_name))
			
	key = 0
	i = 0
	cv2.namedWindow("IMG",0)
	image1_buf_len = len(image1_buf)
	while(key != ord('q')):
		
		j = int(len(image2_buf)*i*1.0/len(image1_buf))
		image1 = image1_buf[i]
		image2 = image2_buf[j]
		scale = 0.4
		image2 =cv2.resize(image2,(0,0),fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
		image3 = imageAdd(image1,image2, 1)

		cv2.imshow("IMG",image3)
		
		if(0):
			key = cv2.waitKey(0)
			if(key == 44):
				i = i -1
			elif(key == 46):
				i = i +1
			i = (i+image1_buf_len)%image1_buf_len
		else:
			key = cv2.waitKey(30)
			cv2.imwrite("result/"+str(i)+".jpg", image3, [int( cv2.IMWRITE_JPEG_QUALITY), 100])
			i = i + 1
			if(i >= image1_buf_len):
				return

	#print(len(image1_buf),len(image2_buf))

def test(argv):
	image1 = cv2.imread(argv[1])
	image2 = cv2.imread(argv[2])
	scale = 0.4
	image2 =cv2.resize(image2,(0,0),fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
	
	image3 = imageAdd(image1,image2, 1)
	
	cv2.namedWindow("IMG",0)
	cv2.imshow("IMG",image3)
	cv2.waitKey(0)


def main(argv):
	#test(argv)
	fun()
	
	
if __name__ == "__main__":
	main(sys.argv)
