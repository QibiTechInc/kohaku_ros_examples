# -*- coding: utf-8 -*-
import rospy
import time
from std_srvs.srv import Empty
from hr4c_msgs.srv import (
    SetInt8Array,
    SetJointTrajectory
)


def wait_key_enter():
    input('')


class SetJointTrajectoryExampleNode(object):
    def __init__(self):
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]
        self._SRV_R_SETCONTROLMODE = '/kohaku_right/set_control_mode'
        self._SRV_R_SERVOALLON = '/kohaku_right/servo_all_on'
        self._SRV_R_SETJOINTTRAJECTORY = '/kohaku_right/set_joint_trajectory'
        self._SRV_R_SERVOALLOFF = '/kohaku_right/servo_all_off'

        rospy.init_node('kohaku_set_joint_trajectory_node')

        # set control mode
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._set_control_mode = rospy.ServiceProxy(
            self._SRV_R_SETCONTROLMODE,
            SetInt8Array
        )

        # servo all on
        rospy.wait_for_service(self._SRV_R_SERVOALLON)
        self._servo_allon = rospy.ServiceProxy(
            self._SRV_R_SERVOALLON,
            Empty
        )

        # set joint trajectory
        rospy.wait_for_service(self._SRV_R_SETJOINTTRAJECTORY)
        self._set_joint_trajectory = rospy.ServiceProxy(
            self._SRV_R_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )

        # servo all off
        rospy.wait_for_service(self._SRV_R_SERVOALLOFF)
        self._servo_alloff = rospy.ServiceProxy(
            self._SRV_R_SERVOALLOFF,
            Empty
        )

    def run(self):
        # set control mode to position
        self._set_control_mode([1, 1, 1,
                                1, 1, 1,
                                1, 1])
        time.sleep(0.2)

        # servo all on
        self._servo_allon()
        time.sleep(0.5)

        # set joint trajectory by position mode
        self._set_joint_trajectory(
            [0.0, 0.0, 2.3,
             0.0, 0.84, 0.0,
             0.0, 0.0],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False,
            True
        )

        # set joint trajectory to other position
        self._set_joint_trajectory(
            [0.0, 0.0, 1.57,
             0.0, 0.99, 0.0,
             0.0, 0.0],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False,
            True
        )

        # wait input enter key
        rospy.loginfo("finish by pressing the enter key")
        wait_key_enter()

        # servo all off
        self._servo_alloff()
