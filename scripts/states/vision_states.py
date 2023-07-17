import rospy
import smach

from typing import *

from modules.base_vision import BaseVision

vision = BaseVision()

class QrDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_qr', 'detected', 'target_position'], input_keys = ["turne", "mission"], output_keys = ["turne", "mission"])

    def execute(self, userdata):
        global vision
        if vision.is_qr_detected():
            userdata.turne = vision.get_angle_to_turne(userdata.mission)
            if vision.is_target_position(userdata.mission):
                return 'target_position'
            rospy.logwarn(f"{userdata.turne, userdata.mission}")
            return 'detected'
        return 'wait_for_qr'
    