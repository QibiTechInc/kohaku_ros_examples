# -*- coding: utf-8 -*-
import rospy
import copy
import moveit_commander
import sys


class MotionExampleNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kohaku_moveit_motion_example_node')

        # standard or dualarm
        self._robot_type = rospy.get_param('~robot_type', 'standard')

        if self._robot_type == 'standard':
            group_name = "arm"
            self._move_group = moveit_commander.MoveGroupCommander(group_name)
        elif self._robot_type == 'dualarm':
            group_name_left = "left_arm"
            self._move_group_left = moveit_commander.MoveGroupCommander(group_name_left)
            group_name_right = "right_arm"
            self._move_group_right = moveit_commander.MoveGroupCommander(group_name_right)
        else:
            rospy.logerr("No such robot type: " + self._robot_type)

    def _get_move_group(self, model):
        if model == 'dualarm_l':
            return self._move_group_left
        elif model == 'dualarm_r':
            return self._move_group_right
        elif model == 'standard':
            return self._move_group
        else:
            raise RuntimeError('Invalid model specification')

    def move_arm_by_joint_angle(self, model, arm_joints,
                                acceleration_scale=1.0,
                                velocity_scale=1.0,
                                wait=True):
        move_group = self._get_move_group(model)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        move_group.go(arm_joints, wait=wait)

        current_joints = move_group.get_current_joint_values()
        rospy.loginfo('Current joints: ' + str(current_joints))
        move_group.stop()

    def move_arm_by_pose(self, model, eef_pose,
                         acceleration_scale=1.0,
                         velocity_scale=1.0,
                         wait=True):
        move_group = self._get_move_group(model)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        move_group.set_joint_value_target(eef_pose, True)
        move_group.go(wait=wait)
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        rospy.loginfo('Current pose: ' + str(current_pose))
        move_group.stop()

    def plan_and_execute_motion_by_pose(self, model, eef_pose,
                                        acceleration_scale=1.0,
                                        velocity_scale=1.0,
                                        wait=True):
        move_group = self._get_move_group(model)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        waypoints = []
        waypoints.append(copy.deepcopy(eef_pose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        move_group.execute(plan, wait=wait)

        current_pose = move_group.get_current_pose().pose
        rospy.loginfo('Current pose: ' + str(current_pose))
        move_group.stop()

    def goto_init_pose(self):
        init_pose = [0.0, 0.0, 2.3, 0.0, 0.84, 0.0]
        if self._robot_type == 'standard':
            self.move_arm_by_joint_angle(self._robot_type, init_pose)
        elif self._robot_type == 'dualarm':
            self.move_arm_by_joint_angle(self._robot_type + '_r', init_pose)
            self.move_arm_by_joint_angle(self._robot_type + '_l', init_pose)
        else:
            rospy.logerr("No such robot type: " + self._robot_type)

    def run(self):
        rospy.loginfo('Start example')
        rospy.loginfo('Go to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Move arms by joint angles')

        if self._robot_type == 'standard':
            target_joints = [-0.26, 1.28, 1.41, 0.0, 0.42, 0.0]
            self.move_arm_by_joint_angle(self._robot_type,
                                         target_joints,
                                         velocity_scale=0.1,
                                         wait=True)

            rospy.loginfo('Move arms by eef poses')
            target_pose = self._move_group.get_current_pose().pose
            target_pose.position.x = 0.26817142787107434
            target_pose.position.y = 0.26073143247697594
            target_pose.position.z = 0.4485311839481246
            self.move_arm_by_pose(self._robot_type,
                                  target_pose,
                                  velocity_scale=0.1,
                                  wait=True)

            self.plan_and_execute_motion_by_pose(self._robot_type,
                                                 target_pose,
                                                 velocity_scale=0.1)
        elif self._robot_type == 'dualarm':
            target_joints_r = [0.26, 1.28, 1.41, 0.0, 0.42, 0.0]
            target_joints_l = [-0.26, 1.28, 1.41, 0.0, 0.42, 0.0]
            self.move_arm_by_joint_angle(self._robot_type + '_r',
                                         target_joints_r,
                                         velocity_scale=0.1,
                                         wait=True)
            self.move_arm_by_joint_angle(self._robot_type + '_l',
                                         target_joints_l,
                                         velocity_scale=0.1,
                                         wait=True)

            rospy.loginfo('Move arms by eef poses')
            target_pose_r = self._move_group_right.get_current_pose().pose
            target_pose_r.position.x = 0.3192783210853239
            target_pose_r.position.y = -0.30274685780459354
            target_pose_r.position.z = 0.3599902791611622

            target_pose_l = self._move_group_left.get_current_pose().pose
            target_pose_l.position.x = 0.26817142787107434
            target_pose_l.position.y = 0.26073143247697594
            target_pose_l.position.z = 0.4485311839481246

            self.move_arm_by_pose(self._robot_type + '_r',
                                  target_pose_r,
                                  velocity_scale=0.1,
                                  wait=True)
            self.plan_and_execute_motion_by_pose(self._robot_type + '_l',
                                                 target_pose_l,
                                                 velocity_scale=0.1)
        else:
            rospy.logerr("No such robot type: " + self._robot_type)

        rospy.loginfo('Return to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Finish example')
        rospy.spin()
