import sys
import cv2
import imageio

def handle(video,start_time,end_time, resolution=1):
	cap = cv2.VideoCapture(video)
	frame_rate = cap.get(cv2.CAP_PROP_FPS)
	frame_cnt = cap.get(cv2.CAP_PROP_FRAME_COUNT)
	total_time = frame_cnt/frame_rate
	if(end_time > total_time):
		end_time = total_time
	start_frame = start_time*frame_rate
	end_frame = end_time*frame_rate
	total_frame = end_frame - start_frame
	
	cap.set(cv2.CAP_PROP_POS_MSEC,start_time*1000)
	
	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	
#	form = cap.get(cv2.CAP_PROP_POS_FRAMES)
#	print("form", form)
	
	fourcc = cv2.VideoWriter_fourcc(*'MP42')
	out = cv2.VideoWriter('1.avi', fourcc, frame_rate, size)
	while True:
		time = cap.get(cv2.CAP_PROP_POS_MSEC)
		if(time >= end_time*1000):
			break
		current_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
		
		ok_frame = current_frame-start_frame
		
		print("%4d/%d" %(ok_frame, total_frame))
		result, img = cap.read()
		if(not result):
			break
		
		if(int(ok_frame)%resolution != 0):
			continue

		out.write(img)
	cap.release()
	out.release()
	print("process ok...")

def main(argv):
	if(len(argv)!=4):
		print("please input video, start time, end time")
		exit()
	handle(argv[1],float(argv[2]),float(argv[3]),5)
	
if __name__ == "__main__":
	main(sys.argv)
