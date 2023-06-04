#!/usr/bin/env python3

# Node that detects the features in the environment through image processing and publishes velocity commands to the move_turtlebot.py node

# import necessary libraries
import rospy
import cv2 
import numpy as np 
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_turtlebot import MoveTurtleBot

class FeatureFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveturtlebot_object = MoveTurtleBot()

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # Cropping to be adjusted
        height, width, channels = cv_image.shape
        cv_image = cv_image[int(height/2):height, 0:width] # height of the image decreased by half and width kept the same to decrease 
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # For yellow color (obtained from hsvcolor_detection.py)
#       low_yellow = np.array([20,100,100])
#       up_yellow = np.array([50,255,255])

        # For red color (obtained from hsvcolor_detection.py)
#       low_red = np.array([0,185,0])
#       up_red = np.array([0,255,255])

        # For green color (obtained from hsvcolor_detection.py)
#       low_red = np.array([45,142,0])
#       up_red = np.array([64,255,255])

        # For blue color (obtained from hsvcolor_detection.py)
        low_blue = np.array([104,134,0]) # Lower blue range
        up_blue = np.array([111,255,255]) # Upper blue range

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, low_blue, up_blue)
        
        # Calculate centroid of the blob of binary image using ImageMoments (built in OpenCV)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        
        # Draw the centroid in the resulting image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RESULT", res)
        
        cv2.waitKey(1)
        
        error_x = cx - width / 2;
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        rospy.loginfo("Angular velocity parameter sent===>"+str(twist_object.angular.z))

        # Make it start turning when no feature is detected [speed is referenced in the MoveTurtleBot() function]moveturtlebot
        self.moveturtlebot_object.move_robot(twist_object) # References the file that moves the turtlebot and publishes commands to the velocity topic
        
    def clean_up(self):
        self.moveturtlebot_object.clean_class()
        cv2.destroyAllWindows()  
        
    # main function runs the quintessential part of the code (Always refer to it as a starting point and work backwards from there)
    def main(): 
        rospy.init_node('follow_feature_node', anonymous=True)
            
        feature_follower_object = FeatureFollower()
    
        rate = rospy.Rate(5)
        ctrl_c = False
        def shutdownhook():
            feature_follower_object.clean_up()
            rospy.loginfo("Time to Shut Down!")
            ctrl_c = True
        
        rospy.on_shutdown(shutdownhook)
        
        while not ctrl_c:
            rate.sleep()

    if __name__ == '__main__':
        main()
