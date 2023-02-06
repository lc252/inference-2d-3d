#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from object_detection.msg import Detection, DetectionsArray



class Detector:
    def __init__(self, render=False):
        # load inference model
        self.model = torch.hub.load("ultralytics/yolov5", "custom", "yolov5n.pt")
        # image subscriber
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.new_image_cb)
        self.img = np.zeros(shape=(480,640,3))
        # timer callback to perform inference. on a separate thread for efficiency
        self.timer = rospy.timer.Timer(rospy.Duration(1/30), self.inference_cb)
        # results publisher
        self.pub = rospy.Publisher('inference_results', DetectionsArray, queue_size=1)
        self.render = render

    def new_image_cb(self, img_msg: Image):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)

    def inference_cb(self, _):
        # do detection
        results = self.model(self.img)
        det_arr = DetectionsArray()
        # publish result ROIs
        if len(results.xyxy[0]) == 0:
            return
        for result in results.xyxy:
            result = result[0]      # tensor containing [xmin, ymin, xmax, ymax, confidence, class]
            det = Detection()
            det.class_id = int(result[5])
            det.confidence = result[4]
            det.roi.x_offset = int(result[0])
            det.roi.y_offset = int(result[1])
            det.roi.height = int(result[3] - result[1])  # ymax - ymin
            det.roi.width = int(result[2] - result[0])   # xmax - xmin
            det.roi.do_rectify = False
            det_arr.objects.append(det)
        
        self.pub.publish(det_arr)
            
        # render result in cv     
        if self.render:
            self.render_cb(results)
    
    def render_cb(self, frame):
        cv2.imshow('Stream', cv2.cvtColor(np.squeeze(frame.render()), cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('yolo_inference_node', anonymous=True)
    display = Detector(render=False)
    rospy.spin()
