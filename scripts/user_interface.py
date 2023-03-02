# -*- coding: utf-8 -*-
"""@package cluedo
This node subscribes to both the state machine and the hint server nodes
and prints their messages on terminal.
"""
import rospy
from std_msgs.msg import String

def callback(msg):
    print(msg.data)
    
    
def main():
    rospy.init_node('user_interface')
    # Intro message
    print("_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")
    print("_-_-_-_-_-_-_-_-_-_-C_L_U_E_D_O-_-_-_-_-_-_-_-_-_")
    print("_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")
    
    # subscribe to the topic /ui_output
    rospy.Subscriber("ui_output", String, callback)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()