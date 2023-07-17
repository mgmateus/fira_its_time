#!/usr/bin/env python3
'''This is an example of how to do mission planning using BaseController. Do not run this module!'''
import rospy

from machines.debbug import *

if __name__ == '__main__':
    rospy.init_node('controller')

    target_height = 0.6

    sm = nave_vision(target_height=target_height)
    #sm = start_land(target_height)
    
    outcome = sm.execute()