import sys
import cv2
import imageio
import os

def handle(path,startSeq,endSeq,suffix,duration):
	
	totalFrameNumber = endSeq - startSeq + 1

	imageBuf = []
	
	for i in range(startSeq,endSeq+1):
		imgName = path + "/" + str(i) + "." + suffix
		if(not os.path.exists(imgName)):
			print("no such file %s" %imgName)
			continue
				
		img = cv2.imread(imgName)
		if(img is None):
			print("read %s failed" %imgName)
			continue
			
		imageBuf.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
		
	outName = 'output.gif'
	imgCnt = len(imageBuf)

	duration = duration*1.0/imgCnt

	print("saving %s ..." %outName)
	gif=imageio.mimsave(outName,imageBuf,'GIF',duration=duration)

	print("%s contains %d frame images, total time is %.2fs" %(outName,imgCnt,duration*imgCnt))

#path startSeq,endSeq,suffix,duration 
def main(argv):
	if(len(argv) < 6):
		print("please input images [path],[startSeq],[endSeq],[suffix],[duration]")
		exit()
	path = argv[1]
	startSeq = int(argv[2])
	endSeq = int(argv[3])
	suffix = argv[4]
	duration = float(argv[5])
	
	handle(path,startSeq,endSeq,suffix,duration)
	
if __name__ == "__main__":
	main(sys.argv)
