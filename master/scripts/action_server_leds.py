#! /usr/bin/python3

import rospy
import actionlib
from actionlib import *
from master.msg import my_TestAction, TestActionFeedback, TestActionResult,_TestActionResult
from actionlib.msg import *

import time
from time import sleep

class OpenDoorServer:

    def __init__(self):
        self.a_server  = actionlib.SimpleActionServer("open_door_leds", my_TestAction, execute_cb = self.execute_cb, auto_start = True)
        self.a_server.start()
        pass

    def execute_cb(self,msg):

        if msg.start == 0:

            print("executing leds")
        time_init = time.time()
        counter = 0
        
        self.a_server.set_succeeded()  # copy that!!  

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

if __name__=='__main__':
    rospy.init_node("door_openning_leds")
    rospy.loginfo('Door opening leds active bro')
    server_2 = OpenDoorServer()
    rospy.spin()