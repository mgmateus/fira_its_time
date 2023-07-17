import rospy
import smach

from typing import *

from modules.base_controller import BaseController

controller = BaseController()
        
class Armed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_arming', 'armed'])

    def execute(self, userdata):
        global controller
        if controller.is_armed():
            return 'armed'
        return 'wait_for_arming'
    

class TakeOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_autonomous_mode', 'wait_for_height'], input_keys = ['target_height'])

    def execute(self, userdata):
        global controller
        if controller.get_mode() != 'STABILIZE':
            controller.takeoff(altitude=userdata.target_height)
            return 'wait_for_height'
        
        return 'wait_for_autonomous_mode'

class Land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['land'])

    def execute(self, userdata):
        global controller
        controller.land()
        return 'land'
    
class WaypointNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint'], input_keys = ['waypoint'])

    def execute(self, userdata):
        global controller
        x, y, z = userdata.waypoint 
        controller.set_position(position_x= x, position_y= y, position_z= z)
        return 'wait_for_waypoint'
    
class StepNavigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint'], input_keys = ['waypoint', 'target_height', 'step'], output_keys = ['waypoint', 'target_height', 'step'])

    def execute(self, userdata):
        global controller
        userdata.waypoint = controller.point_from_distance_rotation(userdata.step, userdata.target_height)
        rospy.logwarn(f"{userdata.waypoint}")
        return 'wait_for_waypoint'
    
class Turnaround(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['wait_for_turne'], input_keys = ['turne'])
        
    def execute(self, userdata):
        global controller
        controller.set_turne(userdata.turne)
        return 'wait_for_turne'
    

class RangeFinderCheck(smach.State):
    def __init__(self, threshold : float = 0.05):
        smach.State.__init__(self, outcomes = ['wait_for_height', 'ready'], input_keys=['target_height'])
        self.__threshold = threshold

    def execute(self, userdata):
        global controller
        if controller.is_target_height(userdata.target_height, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_height'
        
class WaypointCheck(smach.State):
    def __init__(self, threshold : float = 0.1):
        smach.State.__init__(self, outcomes = ['wait_for_waypoint', 'ready'], input_keys = ['waypoint'])
        self.__threshold = threshold

    def execute(self, userdata):
        global controller
        if controller.is_target_position(userdata.waypoint, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_waypoint'
        
class TurnaroundCheck(smach.State):
    def __init__(self, threshold= 5.0):
        smach.State.__init__(self, outcomes = ['wait_for_turne', 'ready'], input_keys = ['turne'])
        self.__threshold = threshold

    def execute(self, userdata):
        global controller
        if controller.is_target_turne(userdata.turne, self.__threshold):
            return 'ready'
        else:
            return 'wait_for_turne'