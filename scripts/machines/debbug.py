import rospy
import smach
import time

from typing import *

from states.controller_states import *
from states.vision_states import *

from machines.base_machines import *
from machines.machines import *


def start(target_height) -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'],
                            input_keys=['target_height', 'saved_waypoint'],
                            output_keys=['target_height', 'saved_waypoint'])
    
    sm.userdata.target_height = target_height
    sm.userdata.saved_waypoint = None
    with sm:
        smach.StateMachine.add('WAIT_ARMED',
                                Armed(),
                                transitions={
                                    'wait_for_arming': 'WAIT_ARMED',
                                    'armed': 'TAKEOFF'
                                })
        
        smach.StateMachine.add('TAKEOFF',
                                TakeOff(),
                                transitions={
                                    'wait_for_autonomous_mode': 'TAKEOFF',
                                    'wait_for_height': 'CHECK_HEIGHT'
                                })
        
        smach.StateMachine.add('CHECK_HEIGHT',
                                RangeFinderCheck(),
                                transitions={
                                    'wait_for_height': 'CHECK_HEIGHT',
                                    'ready': 'succeeded'
                                })
        

    return sm

def start_land(target_height) -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'],
                            input_keys=['target_height'],
                            output_keys=['target_height'])
    
    sm.userdata.target_height = target_height

    with sm:
        sm_start = start(target_height)

        smach.StateMachine.add("FLYING", sm_start,
                                   transitions={
                                        "succeeded" : "LAND"
                                    })

        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })

    return sm

def nave_vision(target_height) -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'],
                            input_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                            output_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'])
    
    sm.userdata.target_height = 1.0
    sm.userdata.step = 0.2
    sm.userdata.turne = 0
    sm.userdata.mission = 1

    with sm:
        sm_start = start(target_height)

        smach.StateMachine.add("FLYING", sm_start,
                                   transitions={
                                        "succeeded" : "NAVE"
                                    })
        
        smach.StateMachine.add('NAVE', StepNavigate(),
                                   transitions={
                                        'wait_for_waypoint' : 'QR_DETECTION'
                                    })
        
        smach.StateMachine.add('QR_DETECTION', QrDetection(),
                                   transitions={
                                        'wait_for_qr' : 'NAVE',
                                        'detected' : 'LAND',
                                        'finished' : 'LAND',
                                    })

        smach.StateMachine.add("LAND",
                               Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm 

def nave_vision_turne(target_height) -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'],
                            input_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                            output_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'])
    
    sm.userdata.target_height = 1.0
    sm.userdata.step = 0.2
    sm.userdata.turne = 0
    sm.userdata.mission = 1

    with sm:
        sm_start = start(target_height)

        smach.StateMachine.add("FLYING", sm_start,
                                   transitions={
                                        "succeeded" : "NAVE"
                                    })
        
        smach.StateMachine.add('NAVE', StepNavigate(),
                                   transitions={
                                        'wait_for_waypoint' : 'WAIT_FOR_QR'
                                    })
        
        con_position_detection = smach.Concurrence(outcomes=['wait_for_detection','turne', 'land'],
                                    default_outcome = 'wait_for_detection',
                                    input_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                                    output_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                                    outcome_map={
                                        'turne': {'QR_DETECTION':'detected'},
                                        'land': {'QR_DETECTION':'finished'}
                                    })
        
        with con_position_detection:
            smach.Concurrence.add('QR_DETECTION', QrDetection())
            smach.Concurrence.add("CHECK_POSITION", WaypointCheck())

        smach.StateMachine.add("WAIT_FOR_QR", con_position_detection,
                                   transitions={
                                        'wait_for_detection' : "NAVE",
                                        'turne' : "TURNE",
                                        'land' : 'LAND'
                                    })

        smach.StateMachine.add("TURNE", Turnaround(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE"
                                })
        
        smach.StateMachine.add("CHECK_TURNE", TurnaroundCheck(),
                                transitions={
                                    "wait_for_turne" : "CHECK_TURNE",
                                    "ready" : "NAVE"
                                })
        
        
        smach.StateMachine.add("LAND", Land(),
                               transitions={
                                   "land": "succeeded"
                               })
        
    return sm 

