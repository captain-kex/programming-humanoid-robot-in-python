'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
import numpy as np # ADDED because it makes things way easier


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None # ADDED

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        # keyframes is basically hello.py and the likes
        # len(keyframes) is 3
        names, times, keys = keyframes
        # len(names)=len(times)=len(keys) is 14 = number of keyframes
        # times are the x-values, keys are the y-values
        # for bezier curves: given: n+1 points -> n is 13 (n+1 is 14)
        # use cubic spline with natural boundary conditions
         
        if self.start_time == None:
            self.start_time = self.perception.time
        # len(keys[x]) is 6
            
        time = self.perception.time - self.start_time
        
        #if time < 0.5:
        #    return target_joints

        for i in range(len(names)): # for every joint do:
            x_values = times[i]
            y_values = []
            # times[i] are the x-values for i-th joint
            # keys[i] are the y-values
            #cubic_spline = natural_cubic_interpolation(times[i], keys[i], time)
            # for spline: y-values: for joint i: keys[i][j][0], j in [0,5]
            for j in range(len(keys[i])):
                y_values.append(keys[i][j][0])
            
            cubic_spline = np.interp(time, x_values, y_values)
            target_joints[names[i]] = cubic_spline
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = rightBackToStand()
    agent.run()
