#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image



class Display:
    def __init__(self):
        # load inference model
        self.model = torch.hub.load("ultralytics/yolov5", "custom", "yolov5n.pt")
        # start a subscriber
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_callback)
        self.img = np.zeros(shape=(640,480,3))
        # timer callback
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.timer_cb)


    def image_callback(self, img_msg: Image):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    def timer_cb(self, image: Image):
        results = self.model(self.img)
        cv2.imshow('Stream', cv2.cvtColor(np.squeeze(results.render()), cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Display()
    rospy.spin()



    
