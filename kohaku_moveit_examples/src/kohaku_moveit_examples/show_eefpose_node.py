# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import sys


class ShowEefposeNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kohaku_moveit_show_eefpose_node')

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

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

    def get_eefpose(self, model):
        move_group = self._get_move_group(model)
        current_pose = move_group.get_current_pose().pose
        return current_pose

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._robot_type == 'standard':
                pose = self.get_eefpose(self._robot_type)
                rospy.loginfo('Current pose: ' + str(pose))
            elif self._robot_type == 'dualarm':
                pose_r = self.get_eefpose(self._robot_type + '_r')
                pose_l = self.get_eefpose(self._robot_type + '_l')
                rospy.loginfo('Current pose(r): ' + str(pose_r))
                rospy.loginfo('Current pose(l): ' + str(pose_l))
            else:
                rospy.logerr("No such robot type: " + self._robot_type)
            r.sleep()
