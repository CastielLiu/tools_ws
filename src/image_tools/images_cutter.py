#coding:utf-8

import sys
import cv2
import imageio
import os

class ImagesCutter:
	def __init__(self):
		self.cut_vertex1 = None
		self.cut_vertex2 = None
		self.selecting = False
		self.cut_ratio = None #长宽比
		self.use_ratio_scale = False
		self.windowName = "IMG"
		self.current_img = None
		self.img_size = None
		self.img_names = []
		self.image_path = None
		self.out_dir = None
		self.inited = False
		self.img_index = 0
		self.select_ok = False
		print("click left button to select mask location.\npress 's' to save new images.\npress 'q' to cancel.")

	def onMouse(self,event,x,y,flags,param):
		
		#已经完成选择,不再进一步处理,防止出错
		if(self.select_ok):
			return
			
		if(x > self.img_size[0]):
			x = self.img_size[0]
		elif(y > self.img_size[1]):
			y = self.img_size[1]
		elif(x < 0):
			x = 0
		elif(y < 0):
			y = 0
	
		if(event == cv2.EVENT_LBUTTONDOWN):
			self.cut_vertex1 = [x,y]
			self.cut_vertex2 = None #此处需将原有顶点2置为无效
			self.selecting = True
		elif(event == cv2.EVENT_LBUTTONUP):
			self.cut_vertex2 = [x,y]
			self.selecting = False
		elif(event == cv2.EVENT_MBUTTONDOWN):
			self.select_ok = True
		elif(event == cv2.EVENT_MOUSEMOVE): #鼠标移动
			if(self.selecting):
				self.cut_vertex2 = [x,y]
		
		if(self.cut_vertex1 and self.cut_vertex2 and self.cut_ratio):
			ratio = 1.0 * self.cut_ratio[0]/self.cut_ratio[1]
			x1,y1 = self.cut_vertex1
			x2,y2 = self.cut_vertex2
			
			if((x2-x1)*(y2-y1)<0):
				self.cut_vertex2[1] = int(y1 - (x2-x1)/ratio)
			else:
				self.cut_vertex2[1] = int(y1 + (x2-x1)/ratio)

	#根据路径、后缀载入所有图片名
	#创建输出文件夹
	def init(self,path,suffix,ratio=None,asTargetScale=False):
		self.cut_ratio = ratio                #裁剪宽高比
		self.use_ratio_scale = asTargetScale  #使用传入的宽高比数值作为目标尺寸
		if(path[-1] == "/"):
			self.image_path = path[0:-1]
		else:
			self.image_path = path
		
		#获取所有可用文件名
		dirs = os.listdir(path)
		for file_ in dirs:
			file_ = os.path.join(path, file_)
			if(not os.path.isfile(file_)):
				continue
			temp = file_.split('.')
			if(temp[-1] != suffix):
				continue
			self.img_names.append(file_)
		
		if(len(self.img_names) == 0):
			print("No image!")
			return False
		
		#创建输出文件夹
		self.out_dir = self.image_path + "_cutted"
		if(not os.path.exists(self.out_dir)):
			os.mkdir(self.out_dir, 0777)
			
		self.inited = True
		return True
		
	def batchCut(self):
		if(self.inited == False):
			print("please initial before cut")
			return
			
		cv2.namedWindow(self.windowName,0)
		cv2.setMouseCallback(self.windowName, self.onMouse)
		
		image_index = 0
		last_image_index = -1
		raw_img = None
		while(not self.select_ok):
			if(image_index != last_image_index):
				raw_img = cv2.imread(self.img_names[image_index])
				last_image_index = image_index
			
			out_img = raw_img.copy() #拷贝一份，防止污染源图
			self.img_size = (out_img.shape[1],out_img.shape[0])
			if(self.cut_vertex1 and self.cut_vertex2): #两顶点同时具备时开始绘制
				point_color = (0, 255, 0) # BGR
				thickness = int(max(self.img_size)/600.0 + 0.5)
				lineType = 3
				cv2.rectangle(out_img, tuple(self.cut_vertex1), tuple(self.cut_vertex2), point_color, thickness, lineType)
					
			cv2.imshow(self.windowName, out_img)
				
			key = cv2.waitKey(30)
			if(ord('q') == key):
				cv2.destroyAllWindows()
				return
			elif(ord('s') == key):
				self.select_ok = True
			elif(ord(',') == key):
				image_index = (image_index+1 + len(self.img_names))%len(self.img_names)
			elif(ord('.') == key):
				image_index = (image_index-1 + len(self.img_names))%len(self.img_names)
			
		cnt = 1
		rowBegin = min(self.cut_vertex1[1], self.cut_vertex2[1])
		rowEnd   = max(self.cut_vertex1[1], self.cut_vertex2[1])
		colBegin = min(self.cut_vertex1[0], self.cut_vertex2[0])
		colEnd   = max(self.cut_vertex1[0], self.cut_vertex2[0])
		
		for img_name in self.img_names:
			img = cv2.imread(img_name)
			if(img is None):
				break
			
			out_img = img[rowBegin:rowEnd,colBegin:colEnd]
			if(self.use_ratio_scale):
				out_img =cv2.resize(out_img,(self.cut_ratio[0],self.cut_ratio[1]),0,0, interpolation=cv2.INTER_NEAREST)
			
			#解算文件名
			temp = img_name.split('/')
			imgRelName = temp[len(temp)-1]
			
			out_name = self.out_dir + "/" + imgRelName
			cv2.imwrite(out_name,out_img)
			print("%d: %s saved." %(cnt,out_name))
			sys.stdout.flush() #强制刷新输出缓冲区
			cnt = cnt + 1
		print("cutted images saved in %s" %self.out_dir)
		sys.stdout.flush() #强制刷新输出缓冲区
	
	
	def oneByoneCut(self):
		if(self.inited == False):
			print("please initial before oneByoneCut")
			return
			
		cv2.namedWindow(self.windowName,0)
		cv2.setMouseCallback(self.windowName, self.onMouse)
		
		image_index = 0
		last_image_index = -1
		raw_img = None
		imgs_handled_flag = [0] * len(self.img_names)
		unHandledImgSize = len(self.img_names)
		
		while(unHandledImgSize):
		
			#查找还未处理的图片索引
			while(imgs_handled_flag[image_index]):
				image_index = (image_index+1+len(self.img_names))%len(self.img_names)
		
			if(image_index != last_image_index):
				raw_img = cv2.imread(self.img_names[image_index])
				last_image_index = image_index
			
			out_img = raw_img.copy() #拷贝一份，防止污染源图
			
			self.img_size = (out_img.shape[1],out_img.shape[0])
			if(self.cut_vertex1 and self.cut_vertex2): #两顶点同时具备时开始绘制
				point_color = (0, 255, 0) # BGR
				thickness = int(max(self.img_size)/600.0 + 0.5)
				lineType = 3
				cv2.rectangle(out_img, tuple(self.cut_vertex1), tuple(self.cut_vertex2), point_color, thickness, lineType)
					
			cv2.imshow(self.windowName, out_img)
				
			key = cv2.waitKey(30)
			if(ord('q') == key):
				cv2.destroyAllWindows()
				return
			elif(ord(',') == key):
				image_index = (image_index+1 + len(self.img_names))%len(self.img_names)
			elif(ord('.') == key):
				image_index = (image_index-1 + len(self.img_names))%len(self.img_names)
			elif(ord('s') == key):
				imgs_handled_flag[image_index] = 1
				unHandledImgSize = unHandledImgSize - 1
			
				rowBegin = min(self.cut_vertex1[1], self.cut_vertex2[1])
				rowEnd   = max(self.cut_vertex1[1], self.cut_vertex2[1])
				colBegin = min(self.cut_vertex1[0], self.cut_vertex2[0])
				colEnd   = max(self.cut_vertex1[0], self.cut_vertex2[0])
			
				out_img = raw_img[rowBegin:rowEnd,colBegin:colEnd]
				if(self.use_ratio_scale):
					out_img =cv2.resize(out_img,(self.cut_ratio[0],self.cut_ratio[1]),0,0, interpolation=cv2.INTER_NEAREST)
		
				#解算文件名
				temp = self.img_names[image_index].split('/')
				imgRelName = temp[len(temp)-1]
		
				out_name = self.out_dir + "/" + imgRelName
				cv2.imwrite(out_name,out_img)
				print("%s saved." %(out_name))
				sys.stdout.flush() #强制刷新输出缓冲区
			
def main(argv):
	if(len(argv) < 4):
		print("please input images [path],[suffix],(mode),(w:h), (asTargetScale)")
		exit()
	path = argv[1]
	suffix = argv[2]
	mode = 0
	ratio = None
	asTargetScale = False
	
	if(len(argv) > 3):
		mode = int(argv[3])
		
	if(len(argv) > 4):
		temp = argv[3].split(':')
		w = int(temp[0])
		h = int(temp[1])
		if(w > 0 and h >0):
			ratio = (w,h)             #裁剪宽高比
		else:
			print("The parameter (w:h) is invalid! ")
	if(len(argv) > 5):
		asTargetScale = bool(argv[5]) #使用传入的宽高比数值作为目标尺寸

	app = ImagesCutter()
	ok = app.init(path,suffix,ratio,asTargetScale)
	if(not ok):
		return
	if(mode == 0):
		app.batchCut()
	else:
		app.oneByoneCut()
	print("over")
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main(sys.argv)
