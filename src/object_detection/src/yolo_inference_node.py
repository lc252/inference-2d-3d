#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from object_detection.msg import Detection



class Detector:
    def __init__(self, render=False):
        # load inference model
        self.model = torch.hub.load("ultralytics/yolov5", "custom", "yolov5n.pt")
        # image subscriber
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.new_image_cb)
        self.img = np.zeros(shape=(640,480,3))
        # timer callback to perform inference. on a separate thread for efficiency
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.inference_cb)
        # results publisher
        self.pub = rospy.Publisher('inference_results', )
        self.render = render

    def new_image_cb(self, img_msg: Image):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    def inference_cb(self):
        # do detection
        results = self.model(self.img)
        # publish result ROIs
        for result in results.xyxy:
            roi = RegionOfInterest()
            roi.x_offset = result[0]    # xmin
            roi.y_offset = result[2]    # ymin
            roi.height = result[3] - result[2]  # ymax - ymin
            roi.width = result[1] - result[0]   # xmax - xmin
            roi.do_rectify = False
            self.pub.publish(roi)
        # render result in cv     
        if self.render:
            self.render_cb(results)
    
    def render_cb(self, frame):
        cv2.imshow('Stream', cv2.cvtColor(np.squeeze(frame.render()), cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Detector(render=True)
    rospy.spin()
