'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent

import numpy as np # ADDED

class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],#, 'LWristYaw', 'LHand'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']#, 'RWristYaw', 'RHand']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)
        
        #[[YXZ, Z, Y, 0],
        # [Z, XYZ, X, 0],
        # [Y, X, XYZ, 0],
        # [X, Y, Z, 1]]
        # upper-left 3x3: Rotaion-matrix (so R_x, R_y or R_z
        # upper right 3x1: Position-vector
        # lower left 1x3: Perspective-transformation
        # lower right 1x1: scale-factor
        
        # from the documentation we know: x is for Roll, y is for Pitch, z is for Yaw
        # we know R_x, R_y and R_z from Lecture Page 35 of 42 and from "rotation matrix" of wikipedia: 
        if 'Roll' in joint_name: # x <-> Roll
            #R_x = [[1, 0, 0],
            #       [0, c, -s],
            #       [0, s, c]]
            T = matrix([[1, 0, 0, 0],
                        [0, c, -s, 0],
                        [0, s, c, 0],
                        [0,0,0,1]])
        elif 'Pitch' in joint_name: # y <-> Pitch
            #R_y = [[c, 0, s],
            #       [0, 1, 0],
            #       [-s, 0, c]]
            T = matrix([[c, 0, s, 0],
                        [0,1,0,0],
                        [-s, 0, c, 0],
                        [0,0,0,1]])
        elif 'Yaw' in joint_name: # z <-> Pitch
            #R_z = [[c, -s, 0],
            #       [s, c, 0],
            #       [0, 0, 1]]
            T = matrix([[c, s, 0, 0],
                        [-s, c, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
            # MAYBE: from the analytical solution for NAOs legs we know that the axis of the hip yaq joints are rotated by 45 degrees
            #if joint_name == 'LHipYawPitch' or joint_name == 'RHipYawPitch':
            #    T_tmp = matrix([[c,0,-s,0], [0,1,0,0], [s,0,c,0], [0,0,0,1]])
            
            # components for translation:
            jointLengths = {'HeadYaw': (0, 0, 126.5), 'HeadPitch': (0, 0, 0),  #head
                            'LShoulderPitch': (0, 98, 100), 'LShoulderRoll': (0, 0, 0), 'LElbowYaw': (105, 15, 0), 'LElbowRoll': (0, 0, 0), #'LWristYaw': (55.95, 0, 0),  #lArm
                            'RShoulderPitch': (0, -98, 100), 'RShoulderRoll': (0, 0, 0), 'RElbowYaw': (105, -15, 0), 'RElbowRoll': (0, 0, 0), #'RWristYaw': (55.95, 0, 0),  #rArm
                            'LHipYawPitch': (0, 50, -85), 'LHipRoll': (0, 0, 0), 'LHipPitch': (0, 0, 0), 'LKneePitch': (0, 0, -100), 'LAnklePitch': (0, 0, -102.9), 'LAnkleRoll': (0, 0, 0),  #lLeg
                            'RHipYawPitch': (0, -50, -85), 'RHipRoll': (0, 0, 0), 'RHipPitch': (0, 0, 0), 'RKneePitch': (0, 0, -100), 'RAnklePitch': (0, 0, -102.9), 'RAnkleRoll': (0, 0, 0)  #rLeg
                            }
            
            T[3, 0] = jointLengths[joint_name][0] #x
            T[3, 1] = jointLengths[joint_name][1] #y
            T[3, 2] = jointLengths[joint_name][2] #z

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T * Tl # multiply old T with current T; * works because Tl of type np.matrix

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
