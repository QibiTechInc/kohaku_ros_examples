# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty


class ServoAllOffExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_servo_alloff_node')

        self._service_name_right = '/kohaku_right/servo_all_off'
        self._service_name_left = '/kohaku_left/servo_all_off'        

        rospy.wait_for_service(self._service_name_right)
        self._servo_alloff_right = rospy.ServiceProxy(
            self._service_name_right,
            Empty
        )
        rospy.wait_for_service(self._service_name_left)
        self._servo_alloff_left = rospy.ServiceProxy(
            self._service_name_left,
            Empty
        )        

    def run(self):
        self._servo_alloff_right()
        self._servo_alloff_left()        
