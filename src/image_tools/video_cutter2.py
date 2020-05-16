#coding:utf-8
import sys
import cv2

def handle(video):
	cap = cv2.VideoCapture(video)
	if(not cap.isOpened()):
		print("open %s failed" %video)
		return

	totalFrameNumber = cap.get(cv2.CAP_PROP_FRAME_COUNT)
	rate = cap.get(cv2.CAP_PROP_FPS)

	print('frames: %f \t frame rate: %f' %(totalFrameNumber,rate))

	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	#fourcc = cv2.VideoWriter_fourcc(*'WMV1')

	fourcc = cv2.VideoWriter_fourcc(*'XVID')
			
	prefix = video.split('.')
	outName = prefix[0]+'_out.'+prefix[1]
	print("output video name: %s" %outName)
	out = cv2.VideoWriter(outName, fourcc, rate, size)
	#print(size)
	start = False
	cv2.namedWindow(video,0)
	while(1):
		ret, frame = cap.read()
	
		current_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
		current_time = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
		print ('current_frame: %d\t current_time: %.1f' %(current_frame,current_frame))
	
		cap.set(cv2.CAP_PROP_POS_FRAMES,current_frame+2)
		if( ret != True):
			break
		
		cv2.imshow(video,frame)
	    
		key = cv2.waitKey(0)
		if(start == False and ord('s') == key):
			start = True
			print("start cut")
		elif ord('q') == key:
			cap.release()
			out.release()
			cv2.destroyAllWindows()
			print("cut over")
			break

		if start:
			out.write(frame)
			print("wrute!")
		

def main(argv):
	if(len(argv)<2):
		print("please input video path!")
		return 
	handle(argv[1])


if __name__ == '__main__':
	try:
		main(sys.argv)
	except KeyboardInterrupt:
		exit()












