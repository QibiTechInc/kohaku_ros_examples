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
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]
        self._SRV_R_SETCONTROLMODE = '/kohaku_right/set_control_mode'
        self._SRV_R_SERVOALLON = '/kohaku_right/servo_all_on'
        self._SRV_R_SETJOINTTRAJECTORY = '/kohaku_right/set_joint_trajectory'
        self._SRV_R_SERVOALLOFF = '/kohaku_right/servo_all_off'
        self._SRV_R_SERVOOFF = '/kohaku_right/servo_off'        

        rospy.init_node('kohaku_mode_change_pos2cur_node')

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

        # servo all off
        rospy.wait_for_service(self._SRV_R_SERVOOFF)
        self._servo_off = rospy.ServiceProxy(
            self._SRV_R_SERVOOFF,
            SetInt8
        )
        
    def run(self):
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
        rospy.loginfo("After grabbing a right arm of the robot, please press the enter key")
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
