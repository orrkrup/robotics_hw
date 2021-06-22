#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import JointState


class FixedJointsPublisher(object):
    def __init__(self, joint_names, joint_values):
        # set the private members for this class
        self.joint_names = joint_names
        self.joint_values = joint_values

        # TODO2: Initialize the node
        rospy.init_node('fixed_joints_publisher', anonymous=False)

        # Create publisher for joint_states topic to publish a fixed joints messages to
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def publish_joints(self):
        new_msg = JointState()
        new_msg.header.stamp = rospy.Time.now()
        # set the joint names and values in the message
        new_msg.name = self.joint_names
        new_msg.position = self.joint_values

        # publish the message
        self.pub.publish(new_msg)

if __name__ == '__main__':
    joint_names = ['base_joint', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3']
    # joint_values = [0., 0., 0., 0., 0., 0., ]
    joint_values = [0.5 * np.pi] * 6
    publisher = FixedJointsPublisher(joint_names, joint_values)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      # invoke the joint publishing method	
      publisher.publish_joints()
      rate.sleep()
    rospy.spin()
