#coding:utf-8

import sys
import os
import shutil
import time

#不同文件夹的图像汇总在一起做gif时,文件编号不能重复，且应按照顺序递增
#使用该工具进行文件编号重新排布

#快排
def quickSort(unsort_array,start,end):
	if(end <= start):
		return
	i = start
	j = end
	key = unsort_array[i]
	
	while(i != j):
		while(unsort_array[j] >= key and j>i):
			j = j - 1
		if(j == i):
			break
		unsort_array[i] = unsort_array[j]
		i = i + 1
		
		while(unsort_array[i] <= key and i<j):
			i = i + 1
		if(i == j):
			break
		unsort_array[j] = unsort_array[i]
		j = j - 1
	unsort_array[j] = key
	
	quickSort(unsort_array,start,j-1)
	quickSort(unsort_array,j+1,end)

def main(argv):
	#print(argv)
	if(len(argv) < 4):
		print("require 3 arguments but %d imput!" %(len(argv)-1))
		print("please input images [path],[suffix],[startSeq]")
		exit()
	images_dir = argv[1]
	if(images_dir[-1] != "/"):
		images_dir += "/"
	suffix = argv[2]
	
	startSeq = int(argv[3])
	
	srcImageNumNames = [] #原文件名,相对名称
	
	#列出路径下的所有文件
	existsFiles = os.listdir(images_dir)
	#查找满足后缀名的文件名,并添加进列表
	for _file in existsFiles:
		splits = _file.split('.')
		if(splits[-1] == suffix):
			try:
				num = int(splits[-2])
			except ValueError:
				continue
			srcImageNumNames.append(num)
			
	imageCnt = len(srcImageNumNames)
	if(imageCnt == 0):
		print("no *.%s images in %s" %(suffix, images_dir))
		sys.stdout.flush()
		return
	
	#文件名按数字大小排序
	quickSort(srcImageNumNames,0,imageCnt-1) 
	
	#先将原文件更名为中间文件，防止名称重复导致为处理的原文件被覆盖
	for i in range(imageCnt):
		src_name = images_dir + str(srcImageNumNames[i]) + "." + suffix
		middle_name = images_dir + "swap_swap_" + str(startSeq+i) + "." + suffix
		os.rename(src_name,middle_name)
	
	#再将中间文件更名为目标名称
	for i in range(imageCnt):
		middle_name = images_dir + "swap_swap_" + str(startSeq+i) + "." + suffix
		dst_name = images_dir + str(startSeq+i) + "." + suffix
		os.rename(middle_name,dst_name)
	
	print("rename %d images complete!" %imageCnt)

def test():
	a = [1,3,5,2,7,6,7,9,-10,5 ,85, 12,99,-8,-5,-9]
	quickSort(a,0,len(a)-1)
	print(a)
	
	
if __name__ == "__main__":
	main(sys.argv)
