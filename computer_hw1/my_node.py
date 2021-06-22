#!/usr/bin/env python

import numpy as np
import rospy
from std_srvs.srv import Empty, Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class Draw(object):
    def __init__(self):
    
        # Initialize the node
        rospy.init_node('drawer', anonymous=False)
        
        # Subscribe to pose topic 
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Subscribe to draw_arch topic
        rospy.Subscriber('/draw_arch', Float32, self.draw_arch_callback)
        
        # Create publisher for cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Create service callback to pause_drawing
        rospy.Service('pause_drawing', Trigger, self.pause_callback)
        
        # Create service callback to resume_drawing
        rospy.Service('resume_drawing', Trigger, self.resume_callback)
        
        # Create handle to the reset service
        self.reset_service = rospy.ServiceProxy('/reset', Empty)

        # Indicator that node is now in the process of drawing
        self.is_busy = False

    def pose_callback(self, msg):
        self.current_angle = msg.theta

    def draw_arch_callback(self, msg):

        # Check availability for drawing a new arch
        if self.is_busy:
            rospy.loginfo('Currently drawing, new request is ignored')
            return

        rospy.loginfo('Recieved draw request')
        self.is_busy = True
        
        # Reset turtlesim
        self.reset_service()
        self.current_angle = None
        
        # Initialize member variables
        self.pose = None
        self.allowed_to_draw = True
        if 0 <= msg.data < np.pi:
            desired_angle = msg.data
        elif np.pi < msg.data <= 2 * np.pi:
            desired_angle = msg.data - 2 * np.pi
        else:
            raise Exception('Angle must be in range [0, 2PI]')
        
        # Wait for first published pose before drawing
        while self.current_angle == None:
            rospy.sleep(0.01)
        rospy.loginfo('Start drawing')
        
        while not rospy.is_shutdown():

            if abs(self.current_angle - desired_angle) < 0.01:
                twist_msg = Twist()
            	twist_msg.linear.x = 0
            	twist_msg.linear.y = 0
            	twist_msg.linear.z = 0
            	twist_msg.angular.x = 0
            	twist_msg.angular.y = 0
            	twist_msg.angular.z = 0
		
		self.cmd_vel_pub.publish(twist_msg)
                break
        
            if not self.allowed_to_draw:
                continue
            
            twist_msg = Twist()
            twist_msg.linear.x = 1
            twist_msg.linear.y = 0
            twist_msg.linear.z = 0
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = 0.5

            rospy.sleep(0.01)
            self.cmd_vel_pub.publish(twist_msg)
         
        rospy.loginfo('Finished drawing')
        self.is_busy = False
   
    def pause_callback(self, reqt):
        self.allowed_to_draw = False
        return TriggerResponse(success=True, message='paused drawing')
   
    def resume_callback(self, req):
        self.allowed_to_draw = True
        return TriggerResponse(success=True, message='resumed drawing')
   
if __name__ == '__main__':
    Draw()
    rospy.spin()
