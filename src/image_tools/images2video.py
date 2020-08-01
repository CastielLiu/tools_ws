#coding=UTF-8

import sys
import cv2
import imageio
import datetime
import os


class Images2Video:
	def init(self, path,startSeq,endSeq,suffix,hz,resolution,scale=None):
		self.imageNames = []
		self.video_size = None
		self.scale = scale
		
		#首先按照seq获取所有可用文件名
		for i in range(startSeq,endSeq+1):
			imgName = path + str(i) + "." + suffix
			if(os.path.exists(imgName)):
				self.imageNames.append(imgName)
		
		self.totalFrameCnt = len(self.imageNames)						#图像总数
		self.needFrameCnt = int(self.totalFrameCnt*1.0/resolution)	#根据取值精度求需要的图像数量
		print("total: %d\t need: %d" %(self.totalFrameCnt, self.needFrameCnt))
		sys.stdout.flush() #强制刷新输出缓冲区
		
		if(self.totalFrameCnt==0 or self.needFrameCnt==0):
			print("no images!")
			sys.stdout.flush() #强制刷新输出缓冲区
			return False
		
		image = cv2.imread(self.imageNames[0])
		if scale is None:
			scale = 1.0
		self.video_size = (int(image.shape[1]*scale), int(image.shape[0]*scale))
		#video_fourcc = cv2.VideoWriter_fourcc(*'MP42')
		video_fourcc = cv2.VideoWriter_fourcc(*'XVID')
		
		now = datetime.datetime.now()
		self.output_name = path[0:-1]+("_%s%s%s-%s%s%s.avi" %(now.year,now.month,now.day,now.hour,now.minute,now.second))
		self.outVideo = cv2.VideoWriter(self.output_name, video_fourcc, hz, self.video_size)
		
		return True

	
	def run(self):
		coef = 1.0*(self.totalFrameCnt-1)/(self.needFrameCnt-1)
		for i in range(self.needFrameCnt):
			index = int(i*coef+0.5)
			imgName = self.imageNames[index]
			img = cv2.imread(imgName)
			if(img is None):
				continue
			if(self.scale):
				img =cv2.resize(img,(0,0),fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)
			print("%.1f%% completed." %(100.0*i/self.needFrameCnt))
			sys.stdout.flush() #强制刷新输出缓冲区
			self.outVideo.write(img)
			
		self.outVideo.release()
		print("cut video complete, saved in %s" %self.output_name)
		sys.stdout.flush() #强制刷新输出缓冲区
		
#path startSeq,endSeq,suffix,hz 
def main(argv):
	if(len(argv) < 6):
		print("please input images [path],[startSeq],[endSeq],[suffix],[hz] (resolution, scale)")
		exit()
	path = argv[1]
	if(path[-1] != "/"):
		path += '/'
		
	startSeq = int(argv[2])
	endSeq = int(argv[3])
	suffix = argv[4]
	hz = float(argv[5])
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
	
	app = Images2Video()
	if(app.init(path,startSeq,endSeq,suffix,hz,resolution,scale)):
		app.run()
	
if __name__ == "__main__":
	main(sys.argv)
	
