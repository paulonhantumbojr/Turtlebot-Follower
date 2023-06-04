#!/usr/bin/env python3

# Publisher Node that moves robot either in a straight line or in a circle using the (Twist) object from the TurtleBot3 library

import rospy # Importing python libraries for ROS
from geometry_msgs.msg import Twist

class MoveTurtleBot(object):

    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publisher to velocity topic of the turtlebot
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False

    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
                                    
    def clean_class(self):
        # Stop Robot when no node is shut down
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_detected = True

def main():
    rospy.init_node('move_turtlebot_node', anonymous=True)
    
    moveturtlebot_object = MoveTurtleBot()
    twist_object = Twist()
    
    # At startup start turning the turtlebot
    twist_object.angular.z = 0.1
    
    rate = rospy.Rate(5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        moveturtlebot_object.clean_class()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        moveturtlebot_object.move_robot(twist_object)
        rate.sleep()

if __name__ == '__main__':
    main()
