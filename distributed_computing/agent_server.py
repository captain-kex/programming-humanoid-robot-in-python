'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys

# own imports:
import numpy as np
import threading # blocking vs non-blocking calls
#import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCServer

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    # it's a class -> need a constructor
    def __init__(self):
        # from https://docs.python.org/2.7/library/simplexmlrpcserver.html
        print('Server initializing')
        super(ServerAgent, self).__init__()
        server_addr = ('0.0.0.0', 4711)
        self.server = SimpleXMLRPCServer(server_addr, allow_none=True) # create a new server instance, allow_none: control the XML-RPC responses that will be returned from the server
        # -> Python constant None will be translated into XML; the default behaviour is for None to raise a TypeError (https://docs.python.org/2.7/library/xmlrpclib.html#module-xmlrpclib)
        
        # register functions:
        self.server.register_function(self.get_angle) # param is the name of the function
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)
        
        target = self.server.serve_forever
        args = []
        self.thread = threading.Thread(target=target, args=args)
        self.thread.start()
        print('Server up and running')
        
        
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        value = self.perception.joint.get(joint_name)
        return value
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.perception.joint[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        #self.start_time = -1
        self.keyframes = keyframes
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transform[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        #self.start_time = -1
        self.set_transform(effector_name, np.matrix(transform))

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

