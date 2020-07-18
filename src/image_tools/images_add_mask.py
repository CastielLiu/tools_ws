#coding:utf-8

import sys
import cv2
import imageio
import os

sys.path.append("lib/")
from image_mask import imageAdd
from image_mask import imageStdAdd

class AddMask:
	def __init__(self):
		self.mask_loc = None
		self.last_mask_loc = None
		self.mask = None
		self.windowName = "IMG"
		self.current_img = None
		self.img_names = []
		self.image_path = None
		self.out_dir = None
		self.inited = False
		self.img_index = 0
		self.save_current = False
		print("click left button to select mask location.\npress 's' to save new images.\npress 'q' to cancel.")

	def onMouse1(self,event,x,y,flags,param):
		if(event == cv2.EVENT_LBUTTONDOWN):
			self.mask_loc = (x,y)
			
	def onMouse2(self,event,x,y,flags,param):
		if(event == cv2.EVENT_LBUTTONDOWN):
			self.mask_loc = (x,y)
		elif(event == cv2.EVENT_MBUTTONDOWN):
			self.save_current = True

	def init(self,path,suffix,mask_image):
		self.image_path = path
		
		self.mask = cv2.imread(mask_image)
		if(self.mask is None):
			print("mask_image is invalid!")
			return False

		#按照seq获取所有可用文件名
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
		self.out_dir = self.image_path + "_masked"
		if(not os.path.exists(self.out_dir)):
			os.mkdir(self.out_dir, 0777)
			
		self.inited = True
		return True
		
	def addMaskInSameLocation(self):
		if(self.inited == False):
			print("please initial before addMaskInSameLocation")
			return
			
		cv2.namedWindow(self.windowName,0)
		cv2.setMouseCallback(self.windowName, self.onMouse1)
		
		raw_img = cv2.imread(self.img_names[0])
		while(True):
			out_img = raw_img.copy() #拷贝一份，防止污染源图
			if(self.mask_loc): #mask位置有效
				out_img = imageAdd(out_img,self.mask, self.mask_loc)
				if(out_img is None): #添加不成功
					self.mask_loc = None #mask位置置为无效
					continue
					
			cv2.imshow(self.windowName, out_img)
				
			key = cv2.waitKey(100)
			if(ord('s') == key):
				break
			elif(ord('q') == key):
				cv2.destroyAllWindows()
				return
		
		new_dir = self.image_path + "_masked"
		if(not os.path.exists(new_dir)):
			os.mkdir(new_dir, 0777)
		
		cnt = 1
		for img_name in self.img_names:
			img = cv2.imread(img_name)
			if(img is None):
				break
			img = imageAdd(img,self.mask, self.mask_loc)
			
			temp = img_name.split('/')
			imgRelName = temp[len(temp)-1]
			
			out_name = new_dir + "/" + imgRelName
			print("%d: %s saved." %(cnt,out_name))
			cnt = cnt + 1
			cv2.imwrite(out_name,img)
			
	def addMaskInDiffLocation(self):
		if(self.inited == False):
			print("please initial before addMaskInDiffLocation")
			return
			
		cv2.namedWindow(self.windowName,0)
		cv2.setMouseCallback(self.windowName, self.onMouse2)
		
		raw_img = None
		while(self.img_index < len(self.img_names)):
			img_name = self.img_names[self.img_index]
			if(raw_img is None): 
				raw_img = cv2.imread(img_name)
				if(raw_img is None): #读取失败,切换到下一张
					self.img_index = self.img_index + 1
					continue;

			out_img = raw_img.copy()#拷贝一份，防止污染源图
			if(self.mask_loc ): #mask位置有效
				out_img = imageAdd(out_img,self.mask, self.mask_loc)
				if(out_img is None):# 添加不成功
					self.mask_loc = None #mask位置置为无效
					continue;
			cv2.imshow(self.windowName, out_img)

			key = cv2.waitKey(100)

			if(ord('q') == key):
				cv2.destroyAllWindows()
				return
			if(self.save_current):
				self.save_current = False
				raw_img = None
				self.img_index = self.img_index + 1
				temp = img_name.split('/')
				imgRelName = temp[-1]
				out_name = self.out_dir + "/" + imgRelName
				print("%s saved." %out_name)
				if(out_img is None):
					print("")
				cv2.imwrite(out_name,out_img);

#mode 0: 所有图片在同一位置添加mask, 点击鼠标左键选择mask位置,位置确定后点击's'开始保存
#mode 1: 每张图片上的mask位置均可指定, 点击鼠标左键选择mask位置,点击鼠标滚轮保存当前图片并切换到下一张
def main(argv):
	if(len(argv) < 5):
		print("please input images [path],[suffix],[mask_image],[mode]")
		exit()
	path = argv[1]
	suffix = argv[2]
	mask_image = argv[3]
	mode = int(argv[4])
	
	app = AddMask()
	ok = app.init(path,suffix,mask_image)
	if(not ok):
		return
	if(mode == 0):
		app.addMaskInSameLocation()
	else:
		app.addMaskInDiffLocation()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main(sys.argv)
