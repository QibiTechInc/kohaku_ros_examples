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
        rospy.init_node('kohaku_set_joint_trajectory_node')

        # vars
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]

        self._robot_type = rospy.get_param('~robot_type', 'standard')
        if self._robot_type == 'standard':
            ns = '/kohaku/'
        elif self._robot_type == 'dualarm':
            ns = '/kohaku_right/'
        else:
            rospy.logerr("No such robot type: " + self._robot_type)
            return

        self._SRV_SETCONTROLMODE = ns + 'set_control_mode'
        self._SRV_SERVOALLON = ns + 'servo_all_on'
        self._SRV_SETJOINTTRAJECTORY = ns + 'set_joint_trajectory'
        self._SRV_SERVOALLOFF = ns + 'servo_all_off'

        # set control mode
        rospy.wait_for_service(self._SRV_SETCONTROLMODE)
        self._set_control_mode = rospy.ServiceProxy(
            self._SRV_SETCONTROLMODE,
            SetInt8Array
        )

        # servo all on
        rospy.wait_for_service(self._SRV_SERVOALLON)
        self._servo_allon = rospy.ServiceProxy(
            self._SRV_SERVOALLON,
            Empty
        )

        # set joint trajectory
        rospy.wait_for_service(self._SRV_SETJOINTTRAJECTORY)
        self._set_joint_trajectory = rospy.ServiceProxy(
            self._SRV_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )

        # servo all off
        rospy.wait_for_service(self._SRV_SERVOALLOFF)
        self._servo_alloff = rospy.ServiceProxy(
            self._SRV_SERVOALLOFF,
            Empty
        )

    def run(self):
        if self._robot_type != 'standard' and self._robot_type != 'dualarm':
            rospy.logerr('Invalid robot type, abort')
            return

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
