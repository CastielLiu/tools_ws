import sys
import cv2
import imageio

def handle(video,resolution,totalTime):
	cap = cv2.VideoCapture(video)
	totalFrameNumber = cap.get(cv2.CAP_PROP_FRAME_COUNT)
	rate = cap.get(cv2.CAP_PROP_FPS)
	print('frames: %f \t frame rate: %f' %(totalFrameNumber,rate))
	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	print(size[0],size[1])
	imageBuf = []
	frameNum = 0.0
	resolution = (totalFrameNumber-1)/(int(totalFrameNumber/resolution)-1)
	while True:
		cap.set(cv2.CAP_PROP_POS_FRAMES,int(frameNum))
		print("current frame index: %d" %cap.get(cv2.CAP_PROP_POS_FRAMES))
		frameNum = frameNum + resolution
		
		result, img = cap.read()
		if(not result):
			break;
		imageBuf.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
		#print(cap.get(cv2.CAP_PROP_POS_FRAMES))
		
	prefix = video.split('.')
	outName = prefix[0]+'.gif'
	imgCnt = len(imageBuf)

	if totalTime is None:
		duration = resolution*1.0/rate
	else:
		duration = totalTime*1.0/imgCnt

	print("saving %s ..." %outName)
	gif=imageio.mimsave(outName,imageBuf,'GIF',duration=duration)

	print("%s contains %d frame images, total time is %.2fs" %(outName,imgCnt,duration*imgCnt))

def main(argv):
	if(len(argv) < 3):
		print("please input a video path and, resolution")
		exit()
	_inVideo = argv[1]
	resolution = int(argv[2])
	if(len(argv) > 3):
		totalTime = float(argv[3])
	else:
		totalTime = None
	handle(_inVideo,resolution,totalTime)
	
if __name__ == "__main__":
	main(sys.argv)
