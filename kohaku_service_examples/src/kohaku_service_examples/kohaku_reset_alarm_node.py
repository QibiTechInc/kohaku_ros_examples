# -*- coding: utf-8 -*-
import rospy
from hr4c_msgs.srv import SetInt8


class ResetAlarmExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_reset_alarm_node')

        self._robot_type = rospy.get_param('~robot_type', 'standard')
        if self._robot_type == 'standard':
            ns = '/kohaku/'
        elif self._robot_type == 'dualarm':
            ns = '/kohaku_right/'
        else:
            rospy.logerr("No such robot type: " + self._robot_type)
            return

        self._service_name = ns + 'alarm_reset'

        rospy.wait_for_service(self._service_name)
        self._reset_alarm = rospy.ServiceProxy(
            self._service_name,
            SetInt8
        )

    def run(self):
        if self._robot_type != 'standard' and self._robot_type != 'dualarm':
            rospy.logerr('Invalid robot type, abort')
            return

        self._reset_alarm(0)
        self._reset_alarm(1)
        self._reset_alarm(2)
        self._reset_alarm(3)
        self._reset_alarm(4)
        self._reset_alarm(5)
