#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

crop_region = [0, 720, 0, 1280]

def depth_callback(data):
    # Convert depth image to NumPy array
    bridge = CvBridge()
    # print(data)
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    depth_image = cv2.flip(depth_image, -1)
    # depth_image = depth_image[crop_region[0]:crop_region[1], crop_region[2]:crop_region[3]]    
    
    # height, width = depth_image.shape[:2]
    # print(height, width)
    # resize to 1280x720
    depth_image = cv2.resize(depth_image, (516,386))
    depth_array = np.array(depth_image, dtype=np.float32)/1000.0

    # Save depth array as .npy file
    np.save('depth_array.npy', depth_array)
    # print(np.unique(depth_array))

class ColorReader(object):
    def __init__(self):
        self.rgb_image = None
        
    def callback(self, data):
        # Convert depth image to NumPy array
        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(data)
        rgb_image = cv2.flip(rgb_image, -1)
        # rgb_image = rgb_image[crop_region[0]:crop_region[1], crop_region[2]:crop_region[3]]
        self.rgb_image = cv2.resize(rgb_image, (516,386))
        # print(crop_region)
        # self.rgb_image = rgb_image
        cv2.imshow('Preview', cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR))
        cv2.imwrite('rgb_image.png', cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



def update_crop_region(x, y, w, h):
    global crop_region
    crop_region = [y, y+h, x, x+w]

def main():
    # Initialize ROS node
    rospy.init_node('depth_saver')
    color_reader = ColorReader()
    # Subscribe to depth image topic
    cv2.namedWindow('Preview', cv2.WINDOW_NORMAL)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, color_reader.callback)
    
    # Create a window for the depth and color images
    # cv2.namedWindow('Preview', cv2.WINDOW_NORMAL)
    # rospy.sleep(1)
    
    
    cv2.createTrackbar('X', 'Preview', crop_region[2], 1280, lambda x: update_crop_region(x, crop_region[0], crop_region[3]-crop_region[2], crop_region[1]-crop_region[0]))
    cv2.createTrackbar('Y', 'Preview', crop_region[0], 720, lambda y: update_crop_region(crop_region[2], y, crop_region[3]-crop_region[2], crop_region[1]-crop_region[0]))
    cv2.createTrackbar('Width', 'Preview', crop_region[3]-crop_region[2], 1280-crop_region[2], lambda w: update_crop_region(crop_region[2], crop_region[0], w, crop_region[1]-crop_region[0]))
    cv2.createTrackbar('Height', 'Preview', crop_region[1]-crop_region[0], 720-crop_region[0], lambda h: update_crop_region(crop_region[2], crop_region[0], crop_region[3]-crop_region[2], h))
    cv2.waitKey()
    # rospy.spin()

if __name__ == '__main__':
    main()
