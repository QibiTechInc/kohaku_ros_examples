# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool
from hr4c_msgs.srv import (
    SetFloat64,
    SetInt8,
    SetInt8Array,
    SetJointTrajectory
)


class MasterSlaveNode(object):
    def __init__(self):
        rospy.init_node('kohaku_master_slave_node')

        # master: left arm, slave: right arm
        self._INTP_LENEAR = 0
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0,
                        0, 0, 0,
                        0, 0]
        self._SRV_L_SETCONTROLMODE = '/kohaku_left/set_control_mode'
        self._SRV_L_SERVOALLON = '/kohaku_left/servo_all_on'
        self._SRV_L_SETJOINTTRAJECTORY = '/kohaku_left/set_joint_trajectory'
        self._SRV_L_SERVOALLOFF = '/kohaku_left/servo_all_off'
        self._SRV_L_SERVOOFF = '/kohaku_left/servo_off'
        self._SRV_L_GO_HOME_POSE = '/kohaku_left/go_to_home_position'
        self._SRV_L_GO_REST_POSE = '/kohaku_left/go_to_rest_position'
        self._SRV_L_ENABLEZEROGMODE = '/kohaku_left/enable_zerog_mode'

        self._SRV_R_SETCONTROLMODE = '/kohaku_right/set_control_mode'
        self._SRV_R_SERVOALLON = '/kohaku_right/servo_all_on'
        self._SRV_R_SETJOINTTRAJECTORY = '/kohaku_right/set_joint_trajectory'
        self._SRV_R_SERVOALLOFF = '/kohaku_right/servo_all_off'
        self._SRV_R_SERVOOFF = '/kohaku_right/servo_off'
        self._SRV_R_GO_HOME_POSE = '/kohaku_right/go_to_home_position'
        self._SRV_R_GO_REST_POSE = '/kohaku_right/go_to_rest_position'
        self._SRV_R_ENABLEZEROGMODE = '/kohaku_right/enable_zerog_mode'
        self._target_angles = None
        self._goal_time = 0.01

        # set control mode
        rospy.wait_for_service(self._SRV_L_SETCONTROLMODE)
        self._set_control_mode_l = rospy.ServiceProxy(
            self._SRV_L_SETCONTROLMODE,
            SetInt8Array
        )
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._set_control_mode_r = rospy.ServiceProxy(
            self._SRV_R_SETCONTROLMODE,
            SetInt8Array
        )

        # servo all on
        rospy.wait_for_service(self._SRV_L_SERVOALLON)
        self._servo_allon_l = rospy.ServiceProxy(
            self._SRV_L_SERVOALLON,
            Empty
        )
        rospy.wait_for_service(self._SRV_R_SERVOALLON)
        self._servo_allon_r = rospy.ServiceProxy(
            self._SRV_R_SERVOALLON,
            Empty
        )

        # set joint trajectory
        rospy.wait_for_service(self._SRV_L_SETJOINTTRAJECTORY)
        self._set_joint_trajectory_l = rospy.ServiceProxy(
            self._SRV_L_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )
        rospy.wait_for_service(self._SRV_R_SETJOINTTRAJECTORY)
        self._set_joint_trajectory_r = rospy.ServiceProxy(
            self._SRV_R_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )

        # servo all off
        rospy.wait_for_service(self._SRV_L_SERVOALLOFF)
        self._servo_alloff_l = rospy.ServiceProxy(
            self._SRV_L_SERVOALLOFF,
            Empty
        )
        rospy.wait_for_service(self._SRV_R_SERVOALLOFF)
        self._servo_alloff_r = rospy.ServiceProxy(
            self._SRV_R_SERVOALLOFF,
            Empty
        )

        # servo off
        rospy.wait_for_service(self._SRV_L_SERVOOFF)
        self._servo_off_l = rospy.ServiceProxy(
            self._SRV_L_SERVOOFF,
            SetInt8
        )
        rospy.wait_for_service(self._SRV_R_SERVOOFF)
        self._servo_off_r = rospy.ServiceProxy(
            self._SRV_R_SERVOOFF,
            SetInt8
        )

        # home
        rospy.wait_for_service(self._SRV_L_GO_HOME_POSE)
        self._go_home_l = rospy.ServiceProxy(
            self._SRV_L_GO_HOME_POSE,
            Empty
        )
        rospy.wait_for_service(self._SRV_R_GO_HOME_POSE)
        self._go_home_r = rospy.ServiceProxy(
            self._SRV_R_GO_HOME_POSE,
            Empty
        )

        # rest
        rospy.wait_for_service(self._SRV_L_GO_REST_POSE)
        self._go_rest_l = rospy.ServiceProxy(
            self._SRV_L_GO_REST_POSE,
            Empty
        )
        rospy.wait_for_service(self._SRV_R_GO_REST_POSE)
        self._go_rest_r = rospy.ServiceProxy(
            self._SRV_R_GO_REST_POSE,
            Empty
        )

        # enable zerog mode
        rospy.wait_for_service(self._SRV_L_SETCONTROLMODE)
        self._enable_zerog_mode_l = rospy.ServiceProxy(
            self._SRV_L_ENABLEZEROGMODE,
            SetBool
        )
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._enable_zerog_mode_r = rospy.ServiceProxy(
            self._SRV_R_ENABLEZEROGMODE,
            SetBool
        )

        rospy.Subscriber("/kohaku_left/joint_states",
                         JointState,
                         self._jointstate_cb)

    def _jointstate_cb(self, msg):
        self._target_angles = msg.position

    def _go_pose_r(self, pose, goal_time):
        self._set_joint_trajectory_r(
            pose,
            goal_time,
            self._NOMASK,
            self._INTP_LENEAR,
            False,
            False
        )

    def initialize(self):
        # servo all on
        self._servo_allon_l()
        self._servo_allon_r()

        # go to home position
        self._go_home_l()
        self._go_home_r()

        # set control mode to torque
        self._set_control_mode_l([4, 4, 4,
                                  4, 4, 4,
                                  1, 1])

        # wait for user to grab master arm
        rospy.loginfo("Grab left arm and pess the enter key")
        input('')

        # servo off master fingers
        self._servo_off_l(6)
        self._servo_off_l(7)

        # set target torque to zero
        self._set_joint_trajectory_l(
            [0,0, 0.0, 0.0,
             0,0, 0.0, 0.0,
             0,0, 0.0],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False,
            True
        )

        # enable zerog mode
        self._enable_zerog_mode_l(True)

    def finalize(self):
        # set control mode to position
        self._set_control_mode_l([1, 1, 1,
                                  1, 1, 1,
                                  1, 1])
        # disable zerog mode
        self._enable_zerog_mode_l(False)

        # set control mode to position
        self._set_control_mode_r([1, 1, 1,
                                  1, 1, 1,
                                  1, 1])

        # go to rest position
        self._go_rest_l()
        self._go_rest_r()
        rospy.loginfo('Finish example motion')

    def run(self):
        self.initialize()
        r = rospy.Rate(int(1/self._goal_time))
        while not rospy.is_shutdown():
            if self._target_angles is not None:
                self._go_pose_r(self._target_angles, self._goal_time)
            r.sleep()
