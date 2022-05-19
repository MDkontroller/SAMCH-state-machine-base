#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from smach_ros import ServiceState, SimpleActionState
#from std_srvs.srv import Trigger    # implement service (nur NOTFALL!!)
#import subprocess

from actionlib_msgs.msg import *
from actionlib import *
from master.msg import my_TestAction, TestActionFeedback, TestActionResult,_TestActionResult

from time import sleep


class Robot_Positioning(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['Initial_Pos'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Robot_Positioning, KUKA')
        rospy.loginfo('------------------------------------------')
        #self.x_pos = 1
        #self.y_pos = 1
        sleep(2)
        # here robot positioning routine KUKA connection
        return 'Initial_Pos'
class Sensors_Com(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes =['Com_Sta'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing comunication')
        rospy.loginfo('------------------------------------------')
        # sensors communication routine
        sleep(2)
        return 'Com_Sta'
class Temperature_Meas(smach.State):


    def __init__(self):
        smach.State.__init__(self, outcomes = ['Temperature'])

    def execute(self, userdata):

        rospy.loginfo('Temperature measumentromt routuine started')
        rospy.loginfo('------------------------------------------')
         
        # subprocess example (parallel) 
        #subprocess.run("python3 /home/nico/catkin_ws/src/master/scripts/test.py & python3 /home/nico/catkin_ws/src/master/scripts/test2.py", shell=True)
        # here routine for measurement.
        sleep(2)
        
        return 'Temperature'

def get_params():

    experiment_name = rospy.get_param("Experiment")
    iterations = rospy.get_param('iterations')
    temperature = rospy.get_param('temperature_start')
    enable_robot = rospy.get_param('list_of_elements')
    some_params = rospy.get_param('Experiment_params')

def main():


    rospy.init_node('main_node')

    get_params()

    #create machine
    my_sm = smach.StateMachine(outcomes =['measurement finished', 'Error','succeeded','aborted','preempted'])

    with my_sm:

        # create sequence
        smach.StateMachine.add('Robot_Pos', Robot_Positioning(),
                                transitions={'Initial_Pos': 'Com'})

        smach.StateMachine.add('Com', Sensors_Com(),
                                transitions={'Com_Sta': 'Temperature_meas' })

        smach.StateMachine.add('Temperature_meas',Temperature_Meas(),
                                transitions={'Temperature':'door_openning_motor'})
                                #transiAttributeError: goal is not an attribute of TestAction
                                #transitions={'Temperature': 'Robot_Pos'})

        # Service client call, calling service "/call"
        # implement service (nur NOTFALL!!)
                 
        #smach.StateMachine.add('Temperature_meas',ServiceState('/call',Trigger),
        #                         transitions={'succeeded':'Robot_Pos'})

        #Action Servers Aufruf ""

        smach.StateMachine.add('door_openning_motor',SimpleActionState('test_action_1',my_TestAction),
                            {'succeeded':'door_openning_leds'})  #{'succeeded':'door_openning_leds'})

        smach.StateMachine.add('door_openning_leds',SimpleActionState('test_action_2',my_TestAction),
                            {'succeeded':'Robot_Pos'})

    # executing Machine sequence
    outcome = my_sm.execute()

    # wait for crtl-c to stop the application
    rospy.spin()
    

if __name__ == '__main__':
    main()
