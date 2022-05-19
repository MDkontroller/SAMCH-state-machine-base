#!/usr/bin/env python3

#from tracemalloc import start
import rospy
import smach
import smach_ros

from smach_ros import ServiceState, SimpleActionState
#from std_srvs.srv import Trigger
import subprocess
from actionlib import *
from actionlib_msgs.msg import *
import time
from time import sleep

from master.msg import my_TestAction, TestActionFeedback, TestActionResult,_TestActionResult



class Robot_Positioning(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['Initial_Pos'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Robot_Positioning, KUKA')
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
        # sensors communication routine
        sleep(2)
        return 'Com_Sta'
class Temperature_Meas(smach.State):


    pass
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Temperature'])

    def execute(self, userdata):

        rospy.loginfo('Temperature measumentromt routuine started')
         
        # subprocess example (parallel) 
        #subprocess.run("python3 /home/nico/catkin_ws/src/master/scripts/test.py & python3 /home/nico/catkin_ws/src/master/scripts/test2.py", shell=True)
        # here routine for measurement.
        sleep(2)
        
        return 'Temperature'
class Leds_Server():

    def __init__(self):

        self.a_server = SimpleActionServer("open_door_server",my_TestAction, execute_cb=self.execute_cb, auto_start = True)
        self.a_server.start()
        rospy.loginfo('Action_Motor_server active bro')

        pass
    def execute_cb(self, start):

        time_init = time.time()
        counter = 0
        self.a_server.set_succeeded()

        while(time.time()-time_init < 4):  # here GPIO mabe, or timer 
            #if self.a_server.is_
            #    sucess = False        
            #   break
            counter = counter + 1
            #print("here sever goes!! ", counter)
            rospy.loginfo('server is running bro! counter: %d',counter)
            rospy.loginfo('------------------------------------------')
            sleep(0.2)
class Motor_server():

    def __init__(self):

        self.a_server =SimpleActionServer('open_door_leds', my_TestAction,execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()
        pass

    def execute_cb(self,start):

        time_init = time.time()
        counter = 0
        
        #self.a_server.set_succeeded()  # copy that!!  

        # Leds  execution::

        while(time.time()-time_init < 6):  # here GPIO mabe, or timer 
            #if self.a_server.is_
            #    sucess = False
            
            #   break
            counter = counter + 1
            rospy.loginfo(' blinking  %d %', counter )
            rospy.loginfo('------------------------------------------')
            sleep(0.2)

        #self.a_server.set_succeeded()


        pass
    pass


def main():
  

    # load params

    rospy.get_params()
    rospy.init_node('Egz_master')

    server_leds  = Leds_Server()
    server_motor = Motor_server()


    #create machine
    my_sm = smach.StateMachine(outcomes =['measurement finished', 'Error','succeeded','aborted','preempted'])

    with my_sm:

        # create sequence
        smach.StateMachine.add('Robot_Pos', Robot_Positioning(),
                                transitions={'Initial_Pos': 'Com'})

        smach.StateMachine.add('Com', Sensors_Com(),
                                transitions={'Com_Sta': 'Temperature_meas' })

        smach.StateMachine.add('Temperature_meas',Temperature_Meas(),
                                transitions={'Temperature':'door_open'})
                                #transiAttributeError: goal is not an attribute of TestAction
                                #transitions={'Temperature': 'Robot_Pos'})

        # Service client call, calling service "/call"
                 
        #smach.StateMachine.add('Temperature_meas',ServiceState('/call',Trigger),
        #                         transitions={'succeeded':'Robot_Pos'})

        #Action Server Aufruf "Action_server_motor.py"

        smach.StateMachine.add('door_open',SimpleActionState('open_door_server',my_TestAction),
                            {'succeeded':'door_openning_leds'})
        
        smach.StateMachine.add('door_openning_leds',SimpleActionState('open_door_leds',my_TestAction),
                            {'succeeded':'Robot_Pos'})

       # goal = my_TestAction(start=1)

    # create introspection server for visualizations
    #sis = smach_ros.IntrospectionServer('Introspection_server', my_sm, '/EGZ_SKYNET')
    #sis.start()

    # executing Machine sequence
    outcome = my_sm.execute()

    # wait for crtl-c to stop the application
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()
