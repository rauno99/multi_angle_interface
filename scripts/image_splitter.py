#!/usr/bin/env python3
import roslib
roslib.load_manifest('multi_angle_interface')
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImageSplitter:
    def __init__(self):
        rospy.init_node('image_splitter', anonymous=True)
        
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("/dual_fisheye_to_panorama/output", Image, self.image_callback)
        self.left_pub = rospy.Publisher("/left_image", Image, queue_size=1)
        self.right_pub = rospy.Publisher("/right_image", Image, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        cv_image = cv2.flip(cv_image, 1)
        height, width, _ = cv_image.shape
        mid_point = width // 2
        left_image = cv_image[:, :mid_point]
        right_image = cv_image[:, mid_point:]

        try:
            left_msg = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_image, "bgr8")
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    img_splitter = ImageSplitter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
