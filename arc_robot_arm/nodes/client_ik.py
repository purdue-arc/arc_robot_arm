#!/usr/bin/python

import zmq
import sys
import numpy as np
import zmqnumpy as znp
from geometry_msgs.msg import Pose 
from sensor_msgs.msg import JointState

def arr2JointState(joint_names, joints):
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = joints

    return joint_state

def pose2arr(pose):
    arr = []
    arr.append(pose.position.x)
    arr.append(pose.position.y)
    arr.append(pose.position.z)
    arr.append(pose.orientation.w)
    arr.append(pose.orientation.x)
    arr.append(pose.orientation.y)
    arr.append(pose.orientation.z)

    return arr

def request_ik_solution(joint_names, goal_pose):
    port = 5556

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:%s" % port)
    
    pose_data = np.array(pose2arr(goal_pose))
    print("Sending pose: ", pose_data,"...")
    
    # Transform array to serialized msg for transport
    pose_msg = znp.array_to_msg(pose_data)
    
    # Send msg
    socket.send_multipart(pose_msg)

    # Get the reply.
    joints = znp.msg_to_array(socket.recv_multipart())
    
    joint_state = arr2JointState(joint_names, joints)

    return joint_state
