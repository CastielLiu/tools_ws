#coding:utf-8
import sys
import cv2

#提供一个图像区域选择接口
class ImageAreaSelecter:
	def __init__(self):
		self.windowName = None
		self.vertex1 = None
		self.vertex2 = None
		self.selecting = False
		self.area_ratio = None #长宽比
		self.img_size = None
		self.inited = False
		self.select_ok = False

	def onMouse(self,event,x,y,flags,param):
		#已经完成选择,不再进一步处理,防止出错
#		if(self.select_ok):
#			return
			
		if(x > self.img_size[0]):
			x = self.img_size[0]
		elif(y > self.img_size[1]):
			y = self.img_size[1]
		elif(x < 0):
			x = 0
		elif(y < 0):
			y = 0
	
		if(event == cv2.EVENT_LBUTTONDOWN):
			self.vertex2 = None #此处需将原有顶点2置为无效
			self.vertex1 = [x,y]
			self.selecting = True
		elif(event == cv2.EVENT_LBUTTONUP):
			#self.vertex2 = [x,y]
			self.selecting = False
#		elif(event == cv2.EVENT_MBUTTONDOWN):
#			self.select_ok = True
		elif(event == cv2.EVENT_MOUSEMOVE): #鼠标移动
			if(self.selecting):
				self.vertex2 = [x,y]
		
		if(self.vertex1 and self.vertex2 and self.area_ratio):
			ratio = 1.0 * self.area_ratio[0]/self.area_ratio[1]
			x1,y1 = self.vertex1
			x2,y2 = self.vertex2
			
			if((x2-x1)*(y2-y1)<0):
				self.vertex2[1] = int(y1 - (x2-x1)/ratio)
			else:
				self.vertex2[1] = int(y1 + (x2-x1)/ratio)
	
	#初始化, windowName: 窗口名称 image_size:图片尺寸  area_ratio:选择区域宽高比
	def init(self,windowName,image_size,area_ratio=None):
		self.windowName = windowName
		self.img_size = image_size
		self.area_ratio = area_ratio
		self.inited = True
		cv2.setMouseCallback(self.windowName, self.onMouse)
		return True
		
	def getAreaVertex(self):
		if(self.vertex1 is None or self.vertex2 is None):
			return None
		if(self.vertex1[1] == self.vertex2[1] or self.vertex1[0] == self.vertex2[0]):
			return None
			
		#区分左上角和右下角
		rowBegin = min(self.vertex1[1], self.vertex2[1]) #left_up_x
		rowEnd   = max(self.vertex1[1], self.vertex2[1]) #right_down_x
		colBegin = min(self.vertex1[0], self.vertex2[0]) #left_up_y
		colEnd   = max(self.vertex1[0], self.vertex2[0]) #right_down_y
		
		return rowBegin,colBegin,rowEnd,colEnd
	
	#绘制选择区域
	def drawSelectTrace(self,image,color=(0,255,0),thickness=2,lineType = 3):
		if(self.vertex1 is None or self.vertex2 is None):
			return image
		if(self.vertex1[1] == self.vertex2[1] or self.vertex1[0] == self.vertex2[0]):
			return image
		cv2.rectangle(image, tuple(self.vertex1), tuple(self.vertex2), color, thickness, lineType)
		return image
		
		
def demo():
	imageAreaSelecter = ImageAreaSelecter()
	imageAreaSelecter.init("windowName",imageSize, areaRatio)
	
	while(True):
		area = imageAreaSelecter.getAreaVertex()
		print(area)
		drawSelectTrace(image)
		
		
