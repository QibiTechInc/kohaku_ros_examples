# -*- coding: utf-8 -*-
import rospy
from hr4c_msgs.srv import SetInt8


class ResetAlarmExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_reset_alarm_node')

        self._service_name_right = '/kohaku_right/alarm_reset'

        rospy.wait_for_service(self._service_name_right)
        self._reset_alarm_right = rospy.ServiceProxy(
            self._service_name_right,
            SetInt8
        )

    def run(self):
        self._reset_alarm_right(0)
        self._reset_alarm_right(1)
        self._reset_alarm_right(2)
        self._reset_alarm_right(3)
        self._reset_alarm_right(4)
        self._reset_alarm_right(5)
