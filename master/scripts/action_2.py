#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from master.msg import my_TestAction
from actionlib import *
from actionlib_msgs.msg import *
from time import sleep

class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                my_TestAction,
                execute_cb=self.execute_cb, auto_start=True)
        rospy.loginfo('node_2 initialized')

    def execute_cb(self, msg):

        time_init = time.time()
        counter = 0
        self._sas.set_succeeded()

        while(time.time()-time_init < 4):  # here GPIO mabe, or timer 
            #if self.a_server.is_
            #    sucess = False        
            #   break
            counter = counter + 1
            #print("here sever goes!! ", counter)
            rospy.loginfo('Testserver is running bro! counter: %d',counter)
            rospy.loginfo('------------------------------------------')
            sleep(0.2)

if __name__=='__main__':
    rospy.init_node("node_2")
    #rospy.loginfo('node_1 active')
    server = TestServer('test_action_2')
    rospy.spin()
