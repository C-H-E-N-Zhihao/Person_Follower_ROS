#!/usr/bin/env python

import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO


class PersonDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)
        
        # YOLO model
        self.model = YOLO("yolov8n.pt")
        self.model.fuse()
        
        # Detection state
        self.person_detected = False
        self.person_center = None
        self.image_width = 640

    def image_callback(self, data):
        """Process camera image and detect person"""
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return
        
        self.image_width = frame.shape[1]
        
        # YOLO detection
        results = self.model(frame, imgsz=self.image_width, verbose=False)[0]
        
        # Find person
        self.person_detected = False
        self.person_center = None
        
        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                confidence = float(box.conf[0])
                
                if self.model.names[cls_id] == 'person' and confidence > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    self.person_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                    self.person_detected = True

                    # Draw box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    break
        
        # Show image
        cv2.imshow("Person Detection", frame)
        cv2.waitKey(1)
    
    def get_person_info(self):
        """Returns (detected, center_x, image_width)"""
        return self.person_detected, self.person_center, self.image_width
    
    def cleanup(self):
        cv2.destroyAllWindows()
