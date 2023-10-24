# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty


class ServoAllOffExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_servo_alloff_node')

        # standard or dualarm
        self._robot_type = rospy.get_param('~robot_type', 'standard')
        if self._robot_type == 'standard':
            self._service_name = '/kohaku/servo_all_off'
            rospy.wait_for_service(self._service_name)
            self._servo_alloff = rospy.ServiceProxy(
                self._service_name,
                Empty
            )
        elif self._robot_type == 'dualarm':
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
        else:
            rospy.logerr("No such robot type: " + self._robot_type)
            return

    def run(self):
        if self._robot_type == 'standard':
            self._servo_alloff()
        elif self._robot_type == 'dualarm':
            self._servo_alloff_right()
            self._servo_alloff_left()
