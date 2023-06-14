# -*- coding: utf-8 -*-
import rospy
import time
from std_srvs.srv import Empty, SetBool
from hr4c_msgs.srv import (
    SetInt8Array,
)    


class EnableZeroGExampleNode(object):
    def __init__(self):
        self._SRV_R_SETCONTROLMODE = '/kohaku_right/set_control_mode'
        self._SRV_R_ENABLEZEROGMODE = '/kohaku_right/enable_zerog_mode'
        self._SRV_R_SERVOALLON = '/kohaku_right/servo_all_on'
        self._SRV_R_SERVOALLOFF = '/kohaku_right/servo_all_off'

        rospy.init_node('kohaku_enable_zerog_node')

        # set control mode
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._set_control_mode = rospy.ServiceProxy(
            self._SRV_R_SETCONTROLMODE,
            SetInt8Array
        )

        # enable zerog mode
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._enable_zerog_mode = rospy.ServiceProxy(
            self._SRV_R_ENABLEZEROGMODE,
            SetBool
        )

        # servo all on
        rospy.wait_for_service(self._SRV_R_SERVOALLON)
        self._servo_allon = rospy.ServiceProxy(
            self._SRV_R_SERVOALLON,
            Empty
        )

        # servo all off
        rospy.wait_for_service(self._SRV_R_SERVOALLOFF)
        self._servo_alloff = rospy.ServiceProxy(
            self._SRV_R_SERVOALLOFF,
            Empty
        )

    def run(self):
        # set control mode to position
        rospy.loginfo("- control mode: TORQUE mode")
        self._set_control_mode([4, 4, 4,
                                4, 4, 4,
                                1, 1])
        time.sleep(0.2)

        # enable zerog mode
        self._enable_zerog_mode(True)

        # servo all on
        rospy.loginfo("- servo ALL ON")
        self._servo_allon()
        time.sleep(0.5)

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("finish by pressing the enter key")
        input('')
        
        # servo all off
        self._servo_alloff()

        # disable zerog mode
        self._enable_zerog_mode(False)

        # return to position control
        self._set_control_mode([1, 1, 1,
                                1, 1, 1,
                                1, 1])
