import rospy
import smach

from smach import CBState
from typing import *

from states.controller_states import *
from states.vision_states import *


def start() -> smach.StateMachine:
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


def nave_avoidance_detection() -> smach.StateMachine:
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
        
        smach.StateMachine.add('NAVE', StepNavigate(),
                                   transitions={
                                        'wait_for_waypoint' : 'WAIT_FOR_QR'
                                    })
        
        con_position_detection = smach.Concurrence(outcomes=['wait_for_detection', 'go_a_head', 'turne', 'land'], #adicionar move_up e move_down
                                    default_outcome = 'wait_for_detection',
                                    input_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                                    output_keys=['waypoint', 'target_height', 'step', 'turne', 'mission'],
                                    outcome_map={
                                        'go_a_head' : {'CHECK_WAYPOINT' : 'ready'},
                                        'turne': {'QR_DETECTION':'detected'},
                                        'target_position': {'QR_DETECTION': 'target_position'}
                                    })
        
        with con_position_detection:
            #adicionar obstacle avoidance
            smach.Concurrence.add('QR_DETECTION', QrDetection())
            smach.Concurrence.add("CHECK_WAYPOINT", WaypointCheck())

        smach.StateMachine.add("WAIT_FOR_QR", con_position_detection,
                                   transitions={
                                        'wait_for_detection' : "WAIT_FOR_QR",
                                        'go_a_head' : "NAVE",
                                        'turne' : "TURNE",
                                        'target_position' : 'succeeded'
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

    return sm

def mission_one() -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add("LAND", 
                               Land(),
                               transitions={
                                   "land": "DELIVERY"
                               })
        
        smach.StateMachine.add('DELIVERY',
                                Delivery(), #implementar
                                transitions={
                                    'done' : 'succeeded'
                                })
        
    return sm  

def mission_two() -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'])
        
    return sm  

def mission_three() -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'])
        
    return sm  

def mission_four() -> smach.StateMachine:
    '''
    Create the start machine to init flight

    Params:
    target_height: altitude to take off

    Returns:
    sm: the machine object machine 
    '''
    sm = smach.StateMachine(outcomes=['succeeded'])
        
    return sm  

def challenge(target_height) -> smach.StateMachine:
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

        smach.StateMachine.add("START", sm_start,
                                   transitions={
                                        "succeeded" : "NAVE_AVOIDANCE_DETECTION"
                                    })
        
        sm_nave_avoidance_detection = nave_avoidance_detection()

        smach.StateMachine.add("NAVE_AVOIDANCE_DETECTION", sm_nave_avoidance_detection,
                                   transitions={
                                        "succeeded" : "CHECK_MISSION"
                                    })

        @smach.cb_interface(input_keys=['mission'], outcomes=['1', '2', '3', '4'])
        def mission_cb(userdata):
            mission = userdata.mission
            userdata.mission += 1
            return mission
        smach.StateMachine.add('CHECK_MISSION', smach.CBState(mission_cb), 
                                    {
                                    "1" : "MISSION_1",
                                    "2" : "MISSION_2",
                                    "3" : "MISSION_3",
                                    "4" : "MISSION_4",
                                    })  
        
        sm_mission_one = mission_one()
        smach.StateMachine.add("MISSION_1", sm_mission_one,
                                   transitions={
                                        "succeeded" : "START"
                                    })
        
        sm_mission_two = mission_two()
        smach.StateMachine.add("MISSION_2", sm_mission_two,
                                   transitions={
                                        "succeeded" : "NAVE"
                                    })
        
        sm_mission_three = mission_three()
        smach.StateMachine.add("MISSION_3", sm_mission_three,
                                   transitions={
                                        "succeeded" : "NAVE"
                                    })
        
        sm_mission_four = mission_four()
        smach.StateMachine.add("MISSION_4", sm_mission_four,
                                   transitions={
                                        "succeeded" : "succeeded"
                                    })