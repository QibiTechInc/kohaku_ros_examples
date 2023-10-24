# -*- coding: utf-8 -*-
import rospy
import time
from std_srvs.srv import Empty, SetBool
from hr4c_msgs.srv import (
    SetInt8Array,
    SetJointTrajectory
)


class EnableZeroGExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_enable_zerog_node')

        self._robot_type = rospy.get_param('~robot_type', 'standard')
        if self._robot_type == 'standard':
            ns = '/kohaku/'
        elif self._robot_type == 'dualarm':
            ns = '/kohaku_right/'
        else:
            rospy.logerr("No such robot type: " + self._robot_type)
            return

        self._SRV_SETCONTROLMODE = ns + 'set_control_mode'
        self._SRV_SETJOINTTRAJECTORY = ns + 'set_joint_trajectory'
        self._SRV_ENABLEZEROGMODE = ns + 'enable_zerog_mode'
        self._SRV_SERVOALLON = ns + 'servo_all_on'
        self._SRV_SERVOALLOFF = ns + 'servo_all_off'

        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]

        # set control mode
        rospy.wait_for_service(self._SRV_SETCONTROLMODE)
        self._set_control_mode = rospy.ServiceProxy(
            self._SRV_SETCONTROLMODE,
            SetInt8Array
        )

        # set joint trajectory
        rospy.wait_for_service(self._SRV_SETJOINTTRAJECTORY)
        self._set_joint_trajectory = rospy.ServiceProxy(
            self._SRV_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )

        # enable zerog mode
        rospy.wait_for_service(self._SRV_SETCONTROLMODE)
        self._enable_zerog_mode = rospy.ServiceProxy(
            self._SRV_ENABLEZEROGMODE,
            SetBool
        )

        # servo all on
        rospy.wait_for_service(self._SRV_SERVOALLON)
        self._servo_allon = rospy.ServiceProxy(
            self._SRV_SERVOALLON,
            Empty
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
        rospy.loginfo("- control mode: TORQUE mode")
        self._set_control_mode([4, 4, 4,
                                4, 4, 4,
                                1, 1])
        time.sleep(0.2)

        # enable zerog mode
        self._enable_zerog_mode(True)

        # set zero reference
        self._set_joint_trajectory(
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0],
            1.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False,
            True
        )

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
