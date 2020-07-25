#coding:utf-8
import sys
import cv2
import imageio
import os

def handle(video):
	cap = cv2.VideoCapture(video)

	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	print(size[0],size[1])
	cnt = 1
	
	prefix,_ = video.split('.')
	images_dir = prefix + "_images"
	if(not os.path.exists(images_dir)):
		os.mkdir(images_dir)
		
	while True:
		result, img = cap.read()
		if(not result):
			break;
		
		img_name = images_dir + "/" + str(cnt) + ".jpg"
		cnt = cnt + 1
		
		#cv2.imwrite(img_name, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
		cv2.imwrite(img_name, img, [int( cv2.IMWRITE_JPEG_QUALITY), 100])
		print("%s saved." %img_name)
		sys.stdout.flush() #强制刷新输出缓冲区
	print("video2images ok\nimages saved in %s" %images_dir)

def main(argv):
	if(len(argv) < 2):
		print("please input video path")
		exit()
	_inVideo = argv[1]
	handle(_inVideo)
	
if __name__ == "__main__":
	main(sys.argv)
