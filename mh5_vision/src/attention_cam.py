#!/usr/bin/env python3

import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class AttentionCam():

    def __init__(self, 
                 device='/dev/video0',
                 height=480,
                 width=640,
                 rate=30.0,
                 scale=4,
                 q_size)

        self.cam = cv2.VideoCapture(device, apiPreference=cv2.CAP_V4L2))

        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cam.set(cv2.CAP_PROP_FPS, rate)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.scaled = (int(width/scale), int(height/scale))

        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('compressed_image', CompressedImage, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/rate), self.publish_img)


    def publish_img(self):

        ret, img = self.cam.read()

        if not ret:
            rospy.logwarn(f'[{rospy.get_name()}] failed to read image from camera')
            return

        if scale == 1:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        else:
            scaled_img = cv2.resize(img, self.scaled, interpolation=cv2.CV_INTER_LINEAR)
            img_msg = self.bridge.cv2_to_compressed_imgmsg(scaled_img)

        self.publisher.publish(img_msg)


if __name__ == '__main__':

    device = rospy.get_param('device', '/dev/video0')
    height = rospy.get_param('image_height', 480)
    width = rospy.get_param('image_width', 640)
    rate = rospy.get_param('rate', 30.0)
    scale = rospy.get_param('scale', 4)
    
    rospy.init_node('camera', log_level=rospy.INFO)

    camera = AttentionCam(device=device,
                          height=height,
                          width=width,
                          rate=rate,
                          scale=scale,
                          q_size=2*rate)

    rospy.spin()
