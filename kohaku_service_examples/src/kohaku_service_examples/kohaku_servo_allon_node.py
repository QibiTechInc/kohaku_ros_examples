# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty


class ServoAllOnExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_servo_allon_node')

        self._service_name_right = '/kohaku_right/servo_all_on'
        self._service_name_left = '/kohaku_left/servo_all_on'        

        rospy.wait_for_service(self._service_name_right)
        self._servo_allon_right = rospy.ServiceProxy(
            self._service_name_right,
            Empty
        )
        rospy.wait_for_service(self._service_name_left)
        self._servo_allon_left = rospy.ServiceProxy(
            self._service_name_left,
            Empty
        )

    def run(self):
        self._servo_allon_right()
        self._servo_allon_left()        
