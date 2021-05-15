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
import numpy as np # ADDED because it makes things way easier
import time # ADDED


def natural_cubic_interpolation(x, y, time):
    """
    Intepolate the given function using a spline with natural boundary conditions.

    Arguments:
    x: x-values of interpolation points
    y: y-values of interpolation points

    Return:
    spline: list of np.poly1d objects, each interpolating the function between two adjacent points
    """
    n = len(x) # len(y) should be the same
    
    if time == 0.0:
        return 0.0
    
    # if time is past the last keyframe:
    if time > x[n-1]:
        return y[n-1][0] # return last y-value for last x-value
    
    # TODO construct linear system with natural boundary conditions

    
    # coefficents
    a = np.array([])
    b = np.array([])
    c = np.array([])
    d = np.array([])
    
    tmp1 = np.array([])
    tmp2 = np.array([])
    tmp3 = np.array([])
    tmp4 = np.array([])
    
    for i in range(n):
        a = np.append(a, y[i][0])
        c = np.append(c, 0)
        tmp3 = np.append(tmp3, 0)
        tmp4 = np.append(tmp4, 0)
        
    for i in range(n-1):
        b = np.append(b, 0)
        d = np.append(d, 0)
        tmp1 = np.append(tmp1, 0)
        tmp2 = np.append(tmp2, x[i+1]-x[i])
        
    arr = [0]
    
    for i in range(1, n-1): # we want to keep arr[0]=0
        arr.append((3/tmp2[i])*(a[i+1]-a[i]) - (3/tmp2[i-1])*(a[i]-a[i-1]))
    
    tmp3[0] = 1
    
    for i in range(1, n-1): # again
        tmp3[i] = 2*(x[i+1]-x[i-1])-tmp2[i-1]*tmp1[i-1]
        tmp1[i] = tmp2[i] / tmp3[i]
        tmp4[i] = (a[i]-tmp2[i-1]*tmp4[i-1])/tmp3[i]
    
    tmp3[n-1] = 1
    
    for i in range(n-2, -1, -1):
        c[i] = tmp4[i] - tmp1[i]*c[i+1]
        b[i] = ((a[i+1]-a[i])/tmp2[i]) - ((tmp2[i]*(c[i+1]+2*c[i]))/3)
        d[i] = (c[i+1] - c[i]) / (3 * tmp2[i])
        
    for i in range(n-1):
        if time < x[i+1]:
            tmp = x[i]
            return a[i] + b[i]*(time-tmp) + c[i]*((time-tmp)**2) + d[i]*((time-tmp)**3)
        
    return y[len(y)-1][0]
        
start_time = -1

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

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
         
        
        # len(keys[x]) is 6
        global start_time
        if start_time == -1:
            start_time = perception.time
            
        time = perception.time - start_time

        for i in range(len(names)): # for every joint do:
            # times[i] are the x-values for i-th joint
            # keys[i] are the y-values
            cubic_spline = natural_cubic_interpolation(times[i], keys[i], time)
            target_joints[names[i]] = cubic_spline
        
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
