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
        #matrix = np.matrix([[0,0,0,0],
        #                    [0,0,0,50],
        #                    [0,0,0,-85],
        #                    [0,0,0,0]])
        #
        #Foot2Hip = transform - matrix # (8.1); type: should be np matrix
        #Foot2HipPositionVector = Foot2Hip[0:3, -1]  # upper right 3x1: Position-vector
        # first problem (axis of hip yaw joints rotated by 45 degrees) solved
        # now the axis of the hip joints can be seen as orthogonal
        # achieved by rotation around x-axis of hip by 45 degrees = pi/4 (360=2pi, 180=pi, 90=pi/2, 45=pi/4)
        # TODO: maybe np.dot
        #tmp = self.local_trans('Roll', np.pi/4)[0:3, 0:3]
        #Foot2HipOrthogonal = np.dot(tmp, Foot2HipPositionVector) # (8.2), 'Roll' for Rotation around x-Axis, pi/4 for 45 degrees
        # Foot2HipOrthogonal = tmp * Foot2HipPositionVector # (8.2), 'Roll' for Rotation around x-Axis, pi/4 for 45 degrees
        
        #HipOrthogonal2Foot = np.linalg.inv(Foot2HipOrthogonal) # (8.3)
        
        # The angle enclosed by the limbs of the leg and the knee corresponds to the knee joint:
        #lUpperLeg = 100
        #lLowerLeg = 102.9
        #lTrans = np.linalg.norm(Foot2HipPositionVector)
        #gamma = np.arccos( (lUpperLeg**2 + lLowerLeg**2 - lTrans**2) / (2 * lUpperLeg * lLowerLeg) ) # (8.5)
        # Because gamma represents an interior angle and the knee joint is being stretched in the zero-position, the resulting angle is computed by:
        #KneePitch = np.pi - gamma # (8.6)
        
        # The angle opposite to the upper leg corresponds to the foot pitch joint
        # footPicht in the paper is AnklePitch in the code
        #AnklePitch1 = np.arccos( (lLowerLeg**2 + lTrans**2 - lUpperLeg**2) / (2 * lLowerLeg * lTrans) ) # (8.7)
        # foot pitch and foot roll joints can be computed with arctan2, x, y and z:
        # x, y and z are the components of the translation of Foot2HipOrthogonal
        #x = Foot2HipOrthogonal[0]
        #norm_y_z = np.linalg.norm(Foot2HipOrthogonal[1:3])
        #y = Foot2HipOrthogonal[1]
        #z = Foot2HipOrthogonal[2]
        #AnklePitch2 = np.arctan2(x, norm_y_z) # (8.8)
        #AnkleRoll = np.arctan2(y, z) # (8.9)
        # foot pitch angle is composed by two parts -> sum of its part:
        #AnklePitch = AnklePitch1 + AnklePitch2 # (8.10)
        
        # Tigh2Foot = Rot_x(delta_AnkleRoll) * Rot_y(delta_AnklePitch) * Trans_z(lLowerLeg) * Rot_y(delta_Knee) * Trans_z(lUpperLeg)
        # x: roll
        # y: pitch
        # z: yaw
        #Rot_x = self.local_trans('Roll', AnkleRoll)
        #Rot_y_1 = self.local_trans('Pitch', AnklePitch)
        #Trans_z_1 = I dont know how to do that
        
        #Foot2Thigh = self.local_trans('LKneePitch', KneePitch).dot(self.local_trans('LAnklePitch', AnklePitch)).dot(self.local_trans('LAnkleRoll', AnkleRoll))
            
        #Hip2HipOrthogonal = np.matlib.identity(4) # instead of np.identity, which would give an ndarray, we get a np matrix
        #x = 1.
        #y = 0.
        #z = 0.
        #c = np.cos(np.pi/4)
        #s = np.sin(np.pi/4)
        #Hip2HipOrthogonalRotation = np.matrix([[(1-c)+c, 0., 0.],
        #                                       [0., c, -s],
        #                                      [0., s, c]]) # calculate the rotation matrix for Hip2HipOrthogonal
        #Hip2HipOrthogonal[0:3, 0:3] = Hip2HipOrthogonalRotation
        
        #Hip2Tigh = Foot2Thigh.dot(np.linalg.inv(Foot2Hip))
        
        #HipYawPitch = np.arctan2(Hip2Tigh[1,0] * np.sqrt(2), Hip2Tigh[1,2])
        
        #def rotation_matrix(angle, vector):
        #    s = np.sin(angle)
        #    c = np.cos(angle)
        #    vector = vector / np.linalg.norm(vector)
        #    x, y, z = vector[0], vector[1], vector[2]
        #    return np.array([[x*x*(1-c)+c,   x*y*(1-c)-z*s, x*z*(1-c)+y*s],
        #                     [y*x*(1-c)+z*s, y*y*(1-c)+c, y*z*(1-c)-x*s],
        #                     [z*x*(1-c)-y*s, z*y*(1-c)+x*s, z*z*(1-c)+c]])
            
            
        #Hip2TighRotation = rotation_matrix(HipYawPitch, [0,1,1])
        #matrix = Hip2Tigh[0:3, 0:3].dot(np.linalg.inv(Hip2TighRotation))
        #HipRoll = np.arctan2(-matrix[1,2], matrix[1,1])
        #HipPitch = np.arctan2(-matrix[2,0], matrix[0,0]) - np.pi
        
        
        # Let's try Jacobian:
        lambda_ = 1 # TODO
        
        # we have the effector_name and get all of it's joints with self.chains[effector_name]:
        all_joints = self.chains[effector_name] # RLeg would give: ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        
        # we want to call forward_kinematics and for that we need a dictionary: joint_angles -> fill joint_angles
        # every key in joint_angles is one element in all_joints
        # we get every corresponding value via self.percection.joint[key]
        for joint in all_joints:
            joint_angles[joint] = self.perception.joint[joint]
            
        # for target:
        # x, y, z from the upper right 3x1: Position-vector
        x = transform.T[0, -1]
        y = transform.T[1, -1]
        z = transform.T[2, -1]
        
        theta_x = np.arctan2(transform.T[2, 1], transform[2, 2])
        theta_y = np.arctan2(-transform.T[2, 0], np.sqrt(transform.T[2, 1]**2 + transform[2, 2]**2))
        theta_z = np.arctan2(transform.T[1,0], transform[0,0])
        
        
        target = np.array([x, y, z, theta_x, theta_y, theta_z])
        
        for i in range(1000):
            self.forward_kinematics(joint_angles) # saved in self.transforms
            Ts = np.zeros(len(self.chains[effector_name]))
            for number, name in enumerate(self.chains[effector_name]):
                Ts[number] = self.transform[name]
                
            # for Te:
            x_e = Ts[-1][0, -1]
            y_e = Ts[-1][1, -1]
            z_e = Ts[-1][2, -1]
            theta_x_e = np.arctan2(Ts[-1][2,1], Ts[-1][2,2])
            theta_y_e = np.arctan2(-Ts[-1][2,0], np.sqrt(Ts[-1][2, 1]**2 + Ts[-1][2, 2]**2))
            theta_z_e = np.arctan2(Ts[-1][1,0], Ts[-1][0,0])
            Te = np.array([x_e, y_e, z_e, theta_x_e, theta_y_e, theta_z_e])
            
            e = target - Te
            
            def get_x_y_z_thetas(m):
                x = m[0,-1]
                y = m[1,-1]
                z = m[2,-1]
                
                theta_x = np.arctan2(m[2,1], m[2,2])
                theta_y = np.arctan2(-m[2,0], np.sqrt(m[2, 1]**2 + m[2, 2]**2))
                theta_z = np.arctan2(m[1,0], m[0,0])
                
                return np.array([x, y, z, theta_x, theta_y, theta_z])
                
            T = np.array([get_x_y_z_thetas(j) for j in Ts[:]])
            
            J = Te - T
            J[:,-1] = 1
            
            d_theta = np.dot(np.dot(J, np.linalg.pinv(np.dot(J.T, J))), e.T) * lambda_
            
            for number, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(d_theta)[number]
            
            if np.linalg.norm(d_theta) < 1e-4:
                break
                
            
        # now we just need to write all angles into joint_angles
        # we can look at the chains for L/RLeg: ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
        #joint_angles = [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]
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
        
        #for chain_name in self.chains:
        #    if (chain_name == 'LLeg'):
        #        i = 0
        #        for joint_name in self.chains['LLeg']:
        #            names.append(joint_name)
        #            times.append([2.0])
        #            keys.append([[joint_angles[i], [3, 0., 0.], [3, 0., 0.]]])
        #            i = i + 1
        #    else:
        #        for joint_name in self.chains[chain_name]:
        #            names.append(joint_name)
        #            times.append([2.0])
        #            keys.append([[0, [3, 0., 0.], [3, 0., 0.]]])
        #for i in range(self.chains[effector_name]):
        #    names.append(self.chains[effector_name][i])
        #    times.append([0,1])
        #    # keys for bezier interpolation:
        #    keys.append([[0, [3, -1e-5, 0.], [3, 1e-5, 1e-5]],
        #                 [joint_angles[i], [3, -1e-5, 1e-5], [3, 1e-5, 1e-5]]])
        #self.set_time(0)
        #self.target_joints.update(joint_angles)
        
        for i, joint_name in enumerate(self.chains[effector_name]):
            names.append(joint_name)
            times.append([1.0, 3.0])
            keys.append([[angles[joint] - 0.01, [3, 0, 0], [3, 0, 0]], [angles[joint], [3, 0, 0], [3, 0, 0]]])
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
