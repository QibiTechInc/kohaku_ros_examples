# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty
from hr4c_msgs.srv import (
    SetFloat64,
    SetInt8Array,
    SetJointTrajectory
)


class PNPGraspNode(object):
    def __init__(self):
        rospy.init_node('kohaku_pnp_grasp_node')

        # vars
        self._INTP_MINJERK = 1
        self._ARMMASK = [0, 0, 0,
                         0, 0, 0,
                         1, 1]

        self._robot_type = rospy.get_param('~robot_type', 'standard')
        if self._robot_type == 'standard':
            ns = '/kohaku/'
        elif self._robot_type == 'dualarm':
            ns = '/kohaku_left/'
        else:
            rospy.logerr("No such robot type: " + self._robot_type)
            return

        self._SRV_SETCONTROLMODE = ns + 'set_control_mode'
        self._SRV_SERVOALLON = ns + 'servo_all_on'
        self._SRV_SETJOINTTRAJECTORY = ns + 'set_joint_trajectory'
        self._SRV_SERVOALLOFF = ns + 'servo_all_off'
        self._SRV_OPENHAND = ns + 'open_hand'
        self._SRV_GRASP = ns + 'grasp'
        self._SRV_GO_HOME_POSE = ns + 'go_to_home_position'
        self._SRV_GO_REST_POSE = ns + 'go_to_rest_position'

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

        # open_hand
        rospy.wait_for_service(self._SRV_OPENHAND)
        self._open_hand = rospy.ServiceProxy(
            self._SRV_OPENHAND,
            SetFloat64
        )

        # grasp
        rospy.wait_for_service(self._SRV_GRASP)
        self._grasp = rospy.ServiceProxy(
            self._SRV_GRASP,
            Empty
        )

        # home
        rospy.wait_for_service(self._SRV_GO_HOME_POSE)
        self._go_home = rospy.ServiceProxy(
            self._SRV_GO_HOME_POSE,
            Empty
        )

        # rest
        rospy.wait_for_service(self._SRV_GO_REST_POSE)
        self._go_rest = rospy.ServiceProxy(
            self._SRV_GO_REST_POSE,
            Empty
        )

    def _go_pose(self, pose, goal_time):
        self._set_joint_trajectory(
            pose,
            goal_time,
            self._ARMMASK,
            self._INTP_MINJERK,
            False,
            True
        )

    def run(self):
        if self._robot_type != 'standard' and self._robot_type != 'dualarm':
            rospy.logerr('Invalid robot type, abort')
            return

        if self._robot_type == 'standard':
            grasping_pose = [-0.3449125238462523, 0.8654321287141796, 2.1238001886254674,
                             -0.3423366365050655, 0.3105860143015352, 0.0,
                             0.0, 0.0]
            target_pose1 = [0.46784095899656863, 0.3708170724432497, 1.7878694566824973,
                            0.0, 0.9469609667231825, 0.0,
                            0.0, 0.0]
            target_pose2 = [-0.04511023938861289, 0.05329871915955386, 2.286063227891548,
                            0.0, 0.8200299518895761, 0.0,
                            0.0, 0.0]
        elif self._robot_type == 'dualarm':
            grasping_pose = [-0.13242437671412624, 1.3349458486899615, 1.8737211890404477,
                             0.0, 0.00103592666903747, 0.0,
                             0.0, 0.0]
            target_pose1 = [0.46784095899656863, 0.3708170724432497, 1.7878694566824973,
                            0.0, 0.9469609667231825, 0.0,
                            0.0, 0.0]
            target_pose2 = [-0.04511023938861289, 0.05329871915955386, 2.286063227891548,
                            0.0, 0.8200299518895761, 0.0,
                            0.0, 0.0]

        # set control mode to position
        self._set_control_mode([1, 1, 1,
                                1, 1, 1,
                                1, 1])
        rospy.sleep(0.2)

        # servo all on
        self._servo_allon()
        rospy.sleep(0.5)

        # go to home position
        self._go_home()

        # open hand
        self._open_hand(0.2)
        rospy.sleep(0.5)

        # go to grasping pose
        self._go_pose(grasping_pose, 3.0)

        # grasp
        rospy.sleep(0.5)
        self._grasp()
        rospy.sleep(1.0)

        # motion loop
        try_count = 3
        for i in range(try_count):
            rospy.loginfo("Count" + str(i))
            self._go_pose(target_pose1, 3.0)
            rospy.sleep(0.5)
            self._go_pose(target_pose2, 3.0)

        # go to grasping pose
        self._go_pose(grasping_pose, 3.0)

        # open hand
        self._open_hand(0.6)

        # go to rest position
        self._go_rest()

        rospy.loginfo('Finish example motion')
