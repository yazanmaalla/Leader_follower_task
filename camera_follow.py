#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Follower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/tb3_1/camera/rgb/image_raw', Image, self.image_callback)
        self.scan_sub = rospy.Subscriber('/tb3_1/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.follower_speed = 0.18
        self.center_error = 0
        self.target_x = 320
        self.target_y = 240
        self.leader_distance = 0.0
        self.distance_threshold = 0.15  # 15 cm in meters

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        mask[0:240, 0:640] = 0
        mask[320:480, 0:640] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.center_error = cx - self.target_x
            cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)
        else:
            self.center_error = 0

        self.twist.linear.x = self.follower_speed
        self.twist.angular.z = -float(self.center_error) / 100
        self.cmd_vel_pub.publish(self.twist)

    def scan_callback(self, data):
        front_scan = data.ranges[0:30] + data.ranges[-30:]  # use the front 60 degrees of the scan
        self.leader_distance = min(front_scan)

    def adjust_speed(self):
        if self.leader_distance > self.distance_threshold:
            self.twist.linear.x = self.follower_speed
        else:
            self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

def main():
    rospy.init_node('follower', anonymous=True)
    follower = Follower()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        follower.adjust_speed()
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

