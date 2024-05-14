#!/usr/bin/env python3
import roslib
roslib.load_manifest('multi_angle_interface')
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageMerger:
    def __init__(self):
        rospy.init_node('dual_fisheye_merger', anonymous=True)
        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Subscribers
        self.left_image_sub = rospy.Subscriber('/kitti360/2d/fisheye/left', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/kitti360/2d/fisheye/right', Image, self.right_image_callback)
        
        # Publishers
        self.image_pub = rospy.Publisher('/dual_fisheye_image', Image, queue_size=10)

        # Variables to hold image data
        self.left_image = None
        self.right_image = None

    def left_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.left_image = cv_image
            self.try_publish()
        except CvBridgeError as e:
            rospy.logerr(e)

    def right_image_callback(self, data):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.right_image = cv_image
            self.try_publish()
        except CvBridgeError as e:
            rospy.logerr(e)

    def try_publish(self):
        if self.left_image is not None and self.right_image is not None:
            # Combine images side by side
            merged_image = cv2.hconcat([self.left_image, self.right_image])

            try:
                # Convert OpenCV image to ROS message and publish
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(merged_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

            # Reset images to ensure synchronization
            self.left_image = None
            self.right_image = None

if __name__ == '__main__':
    image_merger = ImageMerger()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
