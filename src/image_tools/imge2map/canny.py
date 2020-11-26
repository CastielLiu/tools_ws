#!/usr/bin/env python
# coding=utf-8

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

def imagePreProcess(rawImg):
    edges = cv2.Canny(rawImg, 80, 200)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    edges  = cv2.dilate(edges, kernel)

    resultImg = rawImg.copy()
    resultImg[resultImg<50] = 127
    resultImg[edges>200] = 0
    
    #~ cv2.imshow('canny', np.hstack((rawImg, edges)))
    #~ cv2.imshow('resultImg', resultImg)
    #~ cv2.waitKey(0)
    return resultImg

def image2map(image):
    img_size = image.shape
    rows = img_size[0]
    cols = img_size[1]
    
    #print(img_size)
    
    grid_map = OccupancyGrid()
    grid_map.header.frame_id = "map"
    grid_map.info.width = img_size[0]
    grid_map.info.height = img_size[1]
    grid_map.info.resolution = 0.05
    
    width = grid_map.info.width * grid_map.info.resolution
    height = grid_map.info.height * grid_map.info.resolution
    
    grid_map.info.origin.position.x = -width/2
    grid_map.info.origin.position.y = -height/2
    grid_map.info.origin.position.z = 0.0
    grid_map.info.origin.orientation.x = 0.0
    grid_map.info.origin.orientation.y = 0.0
    grid_map.info.origin.orientation.z = 0.0
    grid_map.info.origin.orientation.w = 1.0

    grid_map.data = [0] * (img_size[0]*img_size[1])
    for col in range(cols):
        for row in range(rows):
            index = col*rows + row
            val = image[row, col]
            if(val == 0):
                grid_map.data[index] = 100 #occupy
            elif(val == 127):
                grid_map.data[index] = -1  #unkown
            elif(val == 255):
                grid_map.data[index] = 0   #idle

    return grid_map
    
def main():
    rospy.init_node("image_to_map_node", anonymous=True)
    pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=10)
    
    rawImg = cv2.imread('raw.jpeg', 0)
    map_image = imagePreProcess(rawImg)
    grid_map = image2map(map_image)
    grid_map.header.stamp = rospy.Time.now()
    rate = rospy.Rate(10) # 10hz
    
    while(not rospy.is_shutdown()):
        pub.publish(grid_map)
        rate.sleep()
    
if __name__ == "__main__":
    main()
