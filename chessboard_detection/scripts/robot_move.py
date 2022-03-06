#!/usr/bin/env python

import next_move
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pynput import keyboard

class RobotMove(object):
    def __init__(self):
        self.initial_image = 0
        self.final_image = 0
        self.current_image = 0
        self.is_initial = True
        self.state_file = 'state.txt'
        self.positions = [-1]

        rospy.init_node('robot_move')
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size=1, buff_size=2**24)

    def image_cb(self, data):
        self.current_image = CvBridge.imgmsg_to_cv2(data, "rgb8")
        if self.is_initial:
            self.image_intial = self.current_image
            self.is_initial = False
            print("Press the spacebar once the opponent's move is complete")

    def on_press(key):
        if key == keyboard.Key.space:
            self.final_image = self.current_image
            self.positions = get_next_move(self.initial_image, self. final_image, self.state_file)
            flag = self.send_positions()
            if flag:
                print("Move executed successfully.")
                self.is_initial = True
            else:
                print("An error occured, please undo the move and press the enter key once ready.")

        if key == keyboard.Key.enter:
            self.is_initial = True

    def send_positions():
        flag = False
        if len(self.positions) > 1:
            print(self.positions)
            flag = True
        return flag


if __name__ == '__main__':
    move = RobotMove()
    with Listener(on_press=move.on_press) as listener:
        listener.join()
    rospy.spin()
