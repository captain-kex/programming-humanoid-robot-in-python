'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        
        # let's try it analytically
        # idea from section 8.3.4 B-Human Team Report and Code Release 201 (link in README)
        matrix = np.matrix([[0,0,0,0],
                            [0,0,0,50],
                            [0,0,0,-85],
                            [0,0,0,0]])
        
        Foot2Hip = transform - matrix # (8.1); type: should be np matrix
        Foot2HipPositionVector = Foot2Hip[0:3, -1]  # upper right 3x1: Position-vector
        # first problem (axis of hip yaw joints rotated by 45 degrees) solved
        # now the axis of the hip joints can be seen as orthogonal
        # achieved by rotation around x-axis of hip by 45 degrees = pi/4 (360=2pi, 180=pi, 90=pi/2, 45=pi/4)
        # TODO: maybe np.dot
        tmp = self.local_trans('Roll', np.pi/4)[0:3, 0:3]
        #print(tmp)
        Foot2HipOrthogonal = np.dot(tmp, Foot2HipPositionVector) # (8.2), 'Roll' for Rotation around x-Axis, pi/4 for 45 degrees
        #Foot2HipOrthogonal = tmp * Foot2HipPositionVector # (8.2), 'Roll' for Rotation around x-Axis, pi/4 for 45 degrees
        
        # The angle enclosed by the limbs of the leg and the knee corresponds to the knee joint:
        lUpperLeg = 100
        lLowerLeg = 102.9
        lTrans = np.linalg.norm(Foot2HipPositionVector)
        gamma = np.arccos( (lUpperLeg**2 + lLowerLeg**2 - lTrans**2) / (2 * lUpperLeg * lLowerLeg) ) # (8.5)
        # Because gamma represents an interior angle and the knee joint is being stretched in the zero-position, the resulting angle is computed by:
        KneePitch = np.pi - gamma # (8.6)
        
        # The angle opposite to the upper leg corresponds to the foot pitch joint
        AnklePitch1 = np.arccos( (lLowerLeg**2 + lTrans**2 - lUpperLeg**2) / (2 * lLowerLeg * lTrans) ) # (8.7)
        # foot pitch and foot roll joints can be computed with arctan2, x, y and z:
        # x, y and z are the components of the translation of Foot2HipOrthogonal
        x = Foot2HipOrthogonal[0]
        norm_x_y = np.linalg.norm(Foot2HipOrthogonal[1:3])
        y = Foot2HipOrthogonal[1]
        z = Foot2HipOrthogonal[2]
        AnklePitch2 = np.arctan2(x, norm_x_y) # (8.8)
        AnkleRoll = np.arctan2(y, z) # (8.9)
        # foot pitch angle is composed by two parts -> sum of its part:
        AnklePitch = AnklePitch1 + AnklePitch2 # (8.10)
        
        Foot2Thigh = self.local_trans('LKneePitch', KneePitch).dot(self.local_trans('LAnklePitch', AnklePitch)).dot(self.local_trans('LAnkleRoll', AnkleRoll))
            
        Hip2HipOrthogonal = np.matlib.identity(4) # instead of np.identity, which would give an ndarray, we get a np matrix
        x = 1.
        y = 0.
        z = 0.
        c = np.cos(np.pi/4)
        s = np.sin(np.pi/4)
        Hip2HipOrthogonalRotation = np.matrix([[(1-c)+c, 0., 0.],
                                               [0., c, -s],
                                               [0., s, c]]) # calculate the rotation matrix for Hip2HipOrthogonal
        Hip2HipOrthogonal[0:3, 0:3] = Hip2HipOrthogonalRotation
        
        Hip2Tigh = Foot2Thigh.dot(np.linalg.inv(Foot2Hip))
        
        HipYawPitch = np.arctan2(Hip2Tigh[1,0] * np.sqrt(2), Hip2Tigh[1,2])
        
        def rotation_matrix(angle, vector):
            s = np.sin(angle)
            c = np.cos(angle)
            vector = vector / np.linalg.norm(vector)
            x, y, z = vector[0], vector[1], vector[2]
            
            print('#')
            a = x*x*(1-c)+c
            print(a.shape)
            print(a[0][0][0])
            print('#')
            print((x*y*(1-c)).shape)
            print('#')
            print(x*z*(1-c)+y*s)
            
            
            print('#')
            print(y*x*(1-c)+z*s)
            print('#')
            print(y*y*(1-c)+c)
            print('#')
            print(y*z*(1-c)-x*s)
            
            
            print('#')
            print(z*x*(1-c)-y*s)
            print('#')
            print(z*y*(1-c)+x*s)
            print('#')
            print(z*z*(1-c)+c)
            print('#')
            print('#')
            return np.array([[x*x*(1-c)+c,   x*y*(1-c)-z*s, x*z*(1-c)+y*s],
                             [y*x*(1-c)+z*s, y*y*(1-c)+c, y*z*(1-c)-x*s],
                             [z*x*(1-c)-y*s, z*y*(1-c)+x*s, z*z*(1-c)+c]])
            
            
        Hip2TighRotation = rotation_matrix(HipYawPitch, [0,1,1])
        print(Hip2TighRotation.shape)
        #print(Hip2TighRotation)
        matrix = Hip2Tigh[0:3, 0:3].dot(np.linalg.inv(Hip2TighRotation))
        HipRoll = np.arctan2(-matrix[1,2], matrix[1,1])
        HipPitch = np.arctan2(-matrix[2,0], matrix[0,0]) - np.pi
        
        # now we just need to write all angles into joint_angles
        # we can look at the chains for L/RLeg: ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
        joint_angles = [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]
        return joint_angles
    

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        # we need the same layout as the keyframes from joint_control/keyframes, so three lists:
        names = list()
        times = list()
        keys = list()
        for chain_name in self.chains:
            if (chain_name == 'LLeg'):
                i = 0
                for joint_name in self.chains['LLeg']:
                    names.append(joint_name)
                    times.append([2.0])
                    keys.append([[joint_angles[i], [3, 0., 0.], [3, 0., 0.]]])
                    i = i + 1
            else:
                for joint_name in self.chains[chain_name]:
                    names.append(joint_name)
                    times.append([2.0])
                    keys.append([[0, [3, 0., 0.], [3, 0., 0.]]])
        #for i in range(self.chains[effector_name]):
        #    names.append(self.chains[effector_name][i])
        #    times.append([0,1])
        #    # keys for bezier interpolation:
        #    keys.append([[0, [3, -1e-5, 0.], [3, 1e-5, 1e-5]],
        #                 [joint_angles[i], [3, -1e-5, 1e-5], [3, 1e-5, 1e-5]]])
        #self.set_time(0)
        #self.target_joints.update(joint_angles)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        self.keyframes = (names, times, keys)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
