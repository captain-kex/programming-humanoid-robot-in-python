'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from keyframes import leftBackToStand
import pickle # ADDED, needed for task
from os import listdir # same here
import numpy as np # same here


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER
        ROBOT_POSE_CLF = 'robot_pose.pkl' # from learn_posture.ipnb
        self.posture_classifier = pickle.load(open(ROBOT_POSE_CLF)) # same here

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        # from learn_posture.ipynb we know the feature-list:
        features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'AngleX', 'AngleY']
        features = features[:-2] # all but the last two items. We will use perception.imu instead
        # each pose data (e.g. Back, Belly and so on) has 10 values, each of them corresponding to a feature of features.
        
        ROBOT_POSE_DATA_DIR = 'robot_pose_data' # from learn_posture.ipynb
        classes = listdir(ROBOT_POSE_DATA_DIR) # same here; classes = ['Left', 'Knee', 'Back', 'Right', 'Crouch', 'Sit', 'Frog', 'Stand', 'HeadBack', 'Belly', 'StandInit']; len: 11
        
        #angles = np.zeros(len(features))
        angles = np.array([])
        
        for feature in features:
            angles = np.append(angles, perception.joint[feature])
            
        #angles[-2] = perception.imu[0]
        #angles[-1] = perception.imu[1]
        angles = np.append(angles, perception.imu[0]) # AngleX
        angles = np.append(angles, perception.imu[1]) # AngleY
        #predicted = self.posture_classifier.predict(angles) # does not work because angles is 1d (shape (10,)), not 2d
        angles = np.array([angles]) # shape (1,10)
        #angles = np.reshape(angles, (-1,1)) # automatically set number of rows, but only one column -> shape (10,1)
        predicted = self.posture_classifier.predict(angles) # gives number (as str) from 0 to 10 -> index of classes-array, 7 means: Stand, Robot is standing
        # print(predicted) # DEBUG, can get removed
        posture = classes[int(predicted)]
        # print(posture) # DEBUG, can get removed
        

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = leftBackToStand()
    agent.run()
