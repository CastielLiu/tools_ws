#coding:utf-8
import sys
import cv2

PAUSE = 0
STOP = 1
RUNNING = 2
TEMP_RUN = 3
DISPLAY_CUT_BLOCK = 4
READY_DISPLAY_CUT_BLOCK = 5

class VideoCutter:
	def __init__(self):
		self.cap = None
		self.video_rate = None
		self.video_size = None
		self.video_name = None
		self.setVideoPose = None
		self.currentVideoPose = 0
		self.videoStatus = RUNNING
		self.waitTime = None
		self.cut_begin = 0
		self.cut_end = None
		
		#视频位置滚动条是自动更新还是用户更新
		#自动更新时回调函数只需将此值复位，无需其他操作，方式耗时
		self.posSliderAutoChange = False
		
	def __delete__(self):
		if(self.cap):
			self.cap.release()
	
	def exitProcess(self):
		cv2.destroyAllWindows()
		if(self.cap):
			self.cap.release()
			self.cap = None
		exit(0)
	
	def onVideoPosSliderChanged(self,pos):
		if(self.videoStatus != DISPLAY_CUT_BLOCK):
			cv2.setTrackbarPos('End',self.video_name,pos)
		if(self.posSliderAutoChange):#视频进度条为自动更新,无需调整手动视频位置
			self.posSliderAutoChange = False
			return
		
		#用户拖动了进度条,配置视频位置为滚动条位置
		self.setVideoPose = pos
		#若当前视频处于暂停状态,启动临时播放,以更新当前帧
		if(self.videoStatus == PAUSE):
			self.videoStatus = TEMP_RUN
	
	def onCutBeginSliderChanged(self,pos):
		if(pos > int(self.currentVideoPose)):
			cv2.setTrackbarPos('Beg',self.video_name,int(self.currentVideoPose))
			self.cut_begin = int(self.currentVideoPose)
			return
			
		self.cut_begin = pos

	def onCutEndSliderChanged(self,pos):
		if(pos < int(self.currentVideoPose)):
			cv2.setTrackbarPos('End',self.video_name,int(self.currentVideoPose))
			self.cut_end= int(self.currentVideoPose)
			return
			
		self.cut_end = pos
			
	def cut_video(self, output_name):
		print("start cut video, please wait...")
		sys.stdout.flush()
		#fourcc = cv2.VideoWriter_fourcc(*'WMV1')
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		outVideo = cv2.VideoWriter(output_name, fourcc, self.video_rate, self.video_size)
		
		self.cap.set(cv2.CAP_PROP_POS_FRAMES,self.cut_begin)
		
		while(self.cap.get(cv2.CAP_PROP_POS_FRAMES) <= self.cut_end):
			ret, frame = self.cap.read()
			if( ret != True):
				break
			outVideo.write(frame)
					
		outVideo.release()
		print("cut video complete, saved in %s" %output_name)
		sys.stdout.flush() #强制刷新输出缓冲区
	
	def readyDisplayCutBlock(self):
		#不满足播放条件
		if((self.cut_begin is None) or \
				(self.cut_end is None) or self.cut_begin >= self.cut_end):
			return False
		#已经处于播放状态
		if(self.videoStatus == DISPLAY_CUT_BLOCK):
			self.videoStatus = PAUSE
			return False
			
		self.videoStatus = DISPLAY_CUT_BLOCK
		self.cap.set(cv2.CAP_PROP_POS_FRAMES,self.cut_begin)
		return True
	
	def keyParse(self,key):
		if(ord('s') == key):
			if((self.cut_begin is not None) and \
				(self.cut_end is not None) and self.cut_begin < self.cut_end):
				prefix = self.video_name.split('.')
				output_name = prefix[0]+'_out.'+prefix[1]
				self.cut_video(output_name)
				return False
			else:
				print("please set cut begin and end before click 's'")
				sys.stdout.flush()
		elif(ord('q') == key):
			self.exitProcess()
			return False
		elif(ord('.') == key and self.videoStatus == PAUSE): #down
			self.videoStatus = TEMP_RUN
		elif(ord(',') == key and self.videoStatus == PAUSE): #up
			self.currentVideoPose -= - 2
			if(self.currentVideoPose <=0):
				self.currentVideoPose = 0
			self.cap.set(cv2.CAP_PROP_POS_FRAMES,self.currentVideoPose)
			self.videoStatus = TEMP_RUN
		elif(ord(' ') == key): #按下' '后开始播放视频
			if(self.videoStatus == PAUSE):
				self.videoStatus = RUNNING
			else:
				self.videoStatus = PAUSE
		elif(ord('d') == key): #播放裁剪区
			self.videoStatus = READY_DISPLAY_CUT_BLOCK
		return True

	def handle(self,video_name):
		self.video_name = video_name
		self.cap = cv2.VideoCapture(video_name)
		if(not self.cap.isOpened()):
			print("open %s failed" %video_name)
			return
		
		#获取视频总帧数
		totalFrameNumber = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
		#获取视频帧率
		self.video_rate = self.cap.get(cv2.CAP_PROP_FPS)
		self.waitTime = int(1000.0/self.video_rate)
		
		print('frames: %.1f \t frame rate: %.1f' %(totalFrameNumber,self.video_rate))
		sys.stdout.flush()
		self.video_size = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
		
		cv2.namedWindow(video_name,cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
		#cv2.resizeWindow(video_name,640,480)
		#创建进度条
		cv2.createTrackbar('Beg',video_name,0,int(totalFrameNumber),self.onCutBeginSliderChanged)
		cv2.createTrackbar('Pos',video_name,0,int(totalFrameNumber),self.onVideoPosSliderChanged)
		cv2.createTrackbar('End',video_name,0,int(totalFrameNumber),self.onCutEndSliderChanged)
		cv2.setTrackbarPos("End",video_name,int(totalFrameNumber))
		
		key = -1
		while(1):
			#如果setVideoPose非空,表明外部期望更改视频位置
			if(self.setVideoPose):
				self.cap.set(cv2.CAP_PROP_POS_FRAMES,self.setVideoPose)
				self.setVideoPose = None
			self.currentVideoPose = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
			self.posSliderAutoChange = True
			cv2.setTrackbarPos('Pos',video_name,int(self.currentVideoPose))
			ret, frame = self.cap.read()
			if( ret != True): #视频读取完毕
				self.cap.set(cv2.CAP_PROP_POS_FRAMES,0) #从头播放
				continue
			
			cv2.imshow(self.video_name,frame)
			
			#循环等待进入播放状态
			while(self.videoStatus == PAUSE or self.videoStatus == STOP):
				key = cv2.waitKey(self.waitTime)
				if(not self.keyParse(key)):
					return
			
			key = cv2.waitKey(self.waitTime)
			if(not self.keyParse(key)):
				return
			
			#print(self.videoStatus)
			#正在运行,按帧率等待
			if(self.videoStatus == RUNNING):
				key = cv2.waitKey(self.waitTime)
				if(not self.keyParse(key)):
					return
			#临时播放完毕,暂停播放
			elif(self.videoStatus == TEMP_RUN):
				self.videoStatus = PAUSE
			#准备播放裁剪区, 尝试播放
			elif(self.videoStatus == READY_DISPLAY_CUT_BLOCK):
				if(self.readyDisplayCutBlock()):
					self.videoStatus = DISPLAY_CUT_BLOCK
				else:
					self.videoStatus = RUNNING
			elif(self.videoStatus == DISPLAY_CUT_BLOCK):
				if(self.currentVideoPose == self.cut_end):#裁剪区循环播放
					self.cap.set(cv2.CAP_PROP_POS_FRAMES,self.cut_begin)
				
			
			#current_time = self.cap.get(cv2.CAP_PROP_POS_MSEC)/1000
			#print ('current_frame: %d\t current_time: %.1f' %(current_frame,current_frame))
			
def main(argv):
	if(len(argv)<2):
		print("please input video path!")
		return
	print("press 's' to save cut; press 'q' to exit;    press '>' to next frame;    press '<' to last frame;")
	print("press 'space' to start or stop play;    press 'd' to display cut block;")
	sys.stdout.flush()
	
	app = VideoCutter()
	app.handle(argv[1])

if __name__ == '__main__':
	try:
		main(sys.argv)
	except KeyboardInterrupt:
		exit()













