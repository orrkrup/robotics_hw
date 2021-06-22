#!/usr/bin/env python  

import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

class GeometricServices(object):
    def __init__(self):
        self.current_joints = None
        
        # Subscribe to /joint_states topic
        rospy.Subscriber('/joint_states', JointState, self.joints_callback)

        # Create service callback to tf translations
        self.tf_listener = tf.TransformListener() 
        rospy.Service('get_tf_ee', Trigger, self.get_tf_ee_callback)

        # init your own kinematic chain offsets
        self.a_i = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0] 
        self.alpha_i = [np.pi / 2.0, 0.0, -np.pi / 2.0, np.pi / 2.0, -np.pi / 2.0, 0.0]
        self.d_i = [0.9, 0.0, 0.0, 1.1, 0.05, 0.0]
        self.nue_i = [np.pi / 2.0, np.pi / 2.0, -np.pi / 2.0, 0.0, 0.0, 0.0]

        # Create service callback to ee pose
        self.direct_translation = rospy.Service('get_ee_pose', Trigger, self.get_ee_pose_callback)


    def joints_callback(self, msg):
        self.current_joints = msg.position

    def get_tf_ee_callback(self, reqt):
        try:
            trans, rot = self.tf_listener.lookupTransform('base_link', 'end-effector-link', rospy.Time(0))
            message = 'translation {}, rotation {}'.format(trans, rot)
            return TriggerResponse(success=True, message=message)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return TriggerResponse(success=False, message='Failed, is TF running?')

    def get_ee_pose_callback(self, reqt):
        _, translation, rotation = self._get_ee_pose(self.current_joints)
        message = 'translation {} rotation {}'.format(translation, rotation)
        return TriggerResponse(success=True, message=message)

    def _get_ee_pose(self, joints):
        print(joints)
        # T_0^B
        tobq = np.eye(4)
        # T_EE^6
        tenq = self._generate_homogeneous_transformation(0.0, 0.0, 0.6, -np.pi / 2.0)
        
        theta = np.array(self.nue_i) + np.array(joints)
        # A_i^{i-1}
        A = [self._generate_homogeneous_transformation(*p) for p in 
             zip(self.a_i, self.alpha_i, self.d_i, list(theta))]
        tnoq = reduce(np.dot, A)

        from_base_transform = tobq.dot(tnoq).dot(tenq)
        translation = from_base_transform[:3, 3]
        rotation = self._rotation_to_quaternion(from_base_transform[:3, :3])
        return from_base_transform, translation, rotation

    @staticmethod 
    def _generate_homogeneous_transformation(a, alpha, d, nue):
        cti = np.cos(nue)
        sti = np.sin(nue)
        cai = np.cos(alpha)
        sai = np.sin(alpha)
        htf = np.array([[cti, -sti * cai, sti * sai, a * cti],
                        [sti, cti * cai, -cti * sai, a * sti],
                        [0.0, sai, cai, d],
                        [0.0, 0.0, 0.0, 1.0]])
        return htf

    @staticmethod 
    def _rotation_to_quaternion(r):
        real = np.sqrt(np.trace(r) + 1.0) / 2.0
        q = lambda a, b, c: np.sign(r[a, b] - r[b, a]) * np.sqrt(r[c, c] - r[b, b] - r[a, a] + 1) / 2.0
        x = q(2, 1, 0)
        y = q(0, 2, 1)
        z = q(1, 0, 2)
        return np.array([x, y, z, real])

    def get_geometric_jacobian(self, joints):
        # TODO8
        return None

    def get_analytical_jacobian(self, joints):
        geometric_jacobian = self.get_geometric_jacobian(joints)
        j_p = geometric_jacobian[:3, :]
        j_o = geometric_jacobian[3:, :]
        # TODO9
        return None

    def compute_inverse_kinematics(self, end_pose, max_iterations, error_threshold, time_step, initial_joints, k=1.):
        # TODO10
        return None

    @staticmethod 
    def _normalize_joints(joints):
        res = [j for j in joints]
        for i in range(len(res)):
            res[i] = res[i] + np.pi
            res[i] = res[i] % (2*np.pi)
            res[i] = res[i] - np.pi
        return np.array(res)



def convert_quanternion_to_zyz(q):
    x, y, z, w = q
    r23 = y * z - w * x
    r13 = w * y + x * z
    r33 = w**2 + z**2 - 0.5
    r32 = w * x + y * z
    r31 = x * z - w * y

    phi = np.arctan2(r23, r13)
    theta = np.arctan2(np.sqrt(r13**2, r23**2), r33)
    psi = np.arctan2(r32, -r31)
    
    return [phi, theta, psi] 


def solve_ik(geometric_services):
    end_position = [-0.770, 1.562, 1.050]
    end_zyz = convert_quanternion_to_zyz([0.392, 0.830, 0.337, -0.207])
    end_pose = np.concatenate((end_position, end_zyz), axis=0)
    result = gs.compute_inverse_kinematics(end_pose, max_iterations=10000, error_threshold=0.001, time_step=0.001, initial_joints=[0.1]*6)
    print('ik solution {}'.format(result))


if __name__ == '__main__':
    rospy.init_node('hw2_services_node')
    gs = GeometricServices()
    solve_ik(gs)
    rospy.spin()
