#!/usr/bin/python3

import zmq
import time
import sys
import zmqnumpy as znp
import numpy as np
from ikine_4dof_solver import Ikine4DOFSolver

port = 5556

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:%s" % port)
solver = Ikine4DOFSolver(dof=4)

while True:
    #  Wait for next request from client
    pose_arr = znp.msg_to_array(socket.recv_multipart())

    pose_dict = {}
    pose_dict['position'] = pose_arr[:3]
    pose_dict['orientation'] = pose_arr[3:]
    
    print(pose_dict)

    joints = solver.ikine(pose_dict)
    
    print(joints)

    joints_msg = znp.array_to_msg(joints)
    joints_msg[0] = bytes(joints_msg[0], 'utf-8')

    socket.send_multipart(joints_msg)
