#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image



class Display:
    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "custom", "yolov5n.pt")
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.callback)

    
    def callback(self, image: Image):
        im_arr = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        results = self.model(im_arr)
        cv2.imshow('Stream', cv2.cvtColor(np.squeeze(results.render()), cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Display()
    rospy.spin()



    
