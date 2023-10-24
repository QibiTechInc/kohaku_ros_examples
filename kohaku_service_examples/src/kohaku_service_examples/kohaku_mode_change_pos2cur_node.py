# -*- coding: utf-8 -*-
import rospy
import time
from std_srvs.srv import Empty
from hr4c_msgs.srv import (
    SetInt8,
    SetInt8Array,
    SetJointTrajectory
)


def wait_key_enter():
    input('')


class ChangeModePos2CurExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_mode_change_pos2cur_node')

        # vars
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]

        # standard or dualarm
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
        self._SRV_SERVOOFF = ns + 'servo_off'

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

        # servo all off
        rospy.wait_for_service(self._SRV_SERVOOFF)
        self._servo_off = rospy.ServiceProxy(
            self._SRV_SERVOOFF,
            SetInt8
        )

    def run(self):
        if self._robot_type != 'standard' and self._robot_type != 'dualarm':
            rospy.logerr('Invalid robot type, abort')
            return

        # set control mode to position
        rospy.loginfo("- control mode: POSITION mode")
        self._set_control_mode([1, 1, 1,
                                1, 1, 1,
                                1, 1])
        time.sleep(0.2)

        # servo on for J1, J2, J3
        rospy.loginfo("- servo ALL ON")
        self._servo_allon()
        self._servo_off(3)
        self._servo_off(4)
        self._servo_off(5)
        self._servo_off(6)
        self._servo_off(7)

        time.sleep(0.5)

        # set joint trajectory by position mode
        rospy.loginfo("- joint trajectory")
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

        rospy.loginfo("- reached")

        # wait input enter key
        rospy.loginfo("- ")

        if self._robot_type == 'standard':
            msg = "After grabbing an arm of the robot, please press the enter key"
        elif self._robot_type == 'dualarm':
            msg = "After grabbing a right arm of the robot, please press the enter key"
        rospy.loginfo(msg)
        wait_key_enter()
        rospy.loginfo("- ")

        # change control mode to current, then set joint trajectory
        rospy.loginfo("- change control mode to CURRENT mode from joint 0 to 2")
        self._set_control_mode([3, 3, 3,
                                1, 1, 1,
                                1, 1])
        self._set_joint_trajectory(
            [0.136, -0.6, -1.36,
             0.0, 0.0, 0.0,
             0.0, 0.0],
            1.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False,
            True
        )
        rospy.loginfo("- ok")

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("finish by pressing the enter key")
        wait_key_enter()

        # servo all off
        self._servo_alloff()

        # return to position control
        self._set_control_mode([1, 1, 1,
                                1, 1, 1,
                                1, 1])
