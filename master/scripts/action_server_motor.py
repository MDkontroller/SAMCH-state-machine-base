#! /usr/bin/python3

import rospy
import actionlib
from actionlib import *
from master.msg import my_TestAction, TestActionFeedback, TestActionResult,_TestActionResult
import time
from time import sleep

class OpenDoorServer:

    def __init__(self):

        self.a_server  = actionlib.SimpleActionServer("open_door_server", my_TestAction, execute_cb = self.execute_cb, auto_start = False)
        self.a_server.start()
        pass

    def execute_cb(self,start):
        time_init = time.time()
        counter = 0
        #self.a_server.set_succeeded()  # copy that!!  

        # opening door  execution::

        while(time.time()-time_init < 6):  # here GPIO mabe, or timer 
            #if self.a_server.is_
            #    sucess = False
            
            #   break
            counter = counter + 1
            rospy.loginfo('server is running bro! counter: %d',counter)
            rospy.loginfo(' opening  %d %', counter )
            rospy.loginfo('------------------------------------------')
            sleep(0.2)



if __name__=='__main__':
    rospy.init_node("door_openning")
    rospy.loginfo('Action_Motor_server active bro')
    server = OpenDoorServer()
    rospy.spin()