# -*- coding: utf-8 -*-
import rospy
import copy
from hr4c_msgs.srv import SetJointTrajectory, SetJointTrajectoryRequest
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from moveit_msgs.msg import Constraints, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK


class ConcurrentMotionExampleNode(object):
    def __init__(self):
        rospy.init_node('kohaku_concurrent_motion_example_node')

        # parameters
        self._namespace_left = rospy.get_param('~namespace_left', 'kohaku_left')
        self._namespace_right = rospy.get_param('~namespace_right', 'kohaku_right')

        # attributes
        self._arm_mask = [0, 0, 0,
                          0, 0, 0,
                          1, 1]
        self._gripper_mask = [1, 1, 1,
                              1, 1, 1,
                              0, 0]

    def set_joint_trajectory_srv_client(self,
                                        namespace,
                                        target_angles,
                                        goal_time,
                                        mask,
                                        method,
                                        relative,
                                        wait_interpolation):
        srv_name = namespace + '/set_joint_trajectory'
        rospy.wait_for_service(srv_name)
        try:
            set_joint_trajectory = rospy.ServiceProxy(srv_name,
                                                      SetJointTrajectory)
            set_joint_trajectory(target_angles, goal_time, mask,
                                 method, relative, wait_interpolation)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def go_to_rest_position_srv_client(self, namespace):
        srv_name = namespace + '/go_to_rest_position'
        rospy.wait_for_service(srv_name)
        try:
            go_to_rest_position = rospy.ServiceProxy(srv_name, Empty)
            go_to_rest_position()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def compute_ik_srv_client(self, group_name, target_pose):
        srv_name = '/compute_ik'
        rospy.wait_for_service(srv_name)

        joint_state = rospy.wait_for_message('joint_states', JointState)
        robot_state = RobotState()
        constraints = Constraints()
        robot_state.joint_state = joint_state

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose = target_pose
        try:
            request = PositionIKRequest()
            request.group_name = group_name
            request.robot_state = robot_state
            request.constraints = constraints
            request.pose_stamped = pose_stamped

            compute_ik = rospy.ServiceProxy(srv_name, GetPositionIK)
            response = compute_ik(request)
            if group_name == 'left_arm':
                position = response.solution.joint_state.position[0:8]
            else:
                position = response.solution.joint_state.position[8:]
            return position
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None

    def _get_namespace(self, lr):
        if lr == 'l':
            return self._namespace_left
        elif lr == 'r':
            return self._namespace_right
        else:
            raise RuntimeError('Invalid l/r specification')

    def _get_groupname(self, lr):
        if lr == 'l':
            return 'left_arm'
        elif lr == 'r':
            return 'right_arm'
        else:
            raise RuntimeError('Invalid l/r specification')

    def move_arm_by_joint_angle(self,
                                lr,
                                arm_joints,
                                goal_time):
        namespace = self._get_namespace(lr)
        target_joint_angle = copy.copy(arm_joints)
        target_joint_angle.extend([0.0, 0.0])
        self.set_joint_trajectory_srv_client(namespace,
                                             target_joint_angle,
                                             goal_time,
                                             self._arm_mask,
                                             SetJointTrajectoryRequest.MINJERK,
                                             False,
                                             False)

    def calc_ik_from_pose(self, lr, eef_pose):
        group_name = self._get_groupname(lr)
        target_angles = list(self.compute_ik_srv_client(group_name, eef_pose))
        if len(target_angles) == 8:
            # 最後の回転軸の結果は使わない
            target_angles[5] = 0.0
            return target_angles[:6]
        else:
            rospy.logwarn('Failed to calculate ik')
            return None

    def wait_interpolation(self, goal_time):
        rospy.sleep(goal_time)

    def goto_init_pose(self):
        init_pose = [0.0, 0.0, 2.3, 0.0, 0.84, 0.0]
        goal_time = 3.0
        self.move_arm_by_joint_angle('r', init_pose, goal_time)
        self.move_arm_by_joint_angle('l', init_pose, goal_time)
        self.wait_interpolation(goal_time)

    def goto_rest_pose(self, lr):
        namespace = self._get_namespace(lr)
        self.go_to_rest_position_srv_client(namespace)

    def run(self):
        rospy.loginfo('Start example')
        rospy.loginfo('Go to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Move arms by joint angles')
        target_joints_r = [0.26, 1.28, 1.41, 0.0, 0.42, 0.0]
        target_joints_l = [-0.26, 1.28, 1.41, 0.0, 0.42, 0.0]

        goal_time = 3.0
        self.move_arm_by_joint_angle('r',
                                     target_joints_r,
                                     goal_time)
        self.move_arm_by_joint_angle('l',
                                     target_joints_l,
                                     goal_time)
        self.wait_interpolation(goal_time)

        rospy.loginfo('Move arms by eef poses')
        # base_link座標系でhand座標系の姿勢を指示
        target_pose_r = Pose()
        target_pose_r.position.x = 0.3192783210853239
        target_pose_r.position.y = -0.30274685780459354
        target_pose_r.position.z = 0.3599902791611622

        target_pose_l = Pose()
        target_pose_l.position.x = 0.26817142787107434
        target_pose_l.position.y = 0.26073143247697594
        target_pose_l.position.z = 0.4485311839481246

        target_joints_r = self.calc_ik_from_pose('r', target_pose_r)
        target_joints_l = self.calc_ik_from_pose('l', target_pose_l)

        goal_time = 3.0
        if target_joints_r is not None:
            self.move_arm_by_joint_angle('r',
                                         target_joints_r,
                                         goal_time)
        if target_joints_l is not None:
            self.move_arm_by_joint_angle('l',
                                         target_joints_l,
                                         goal_time)
        self.wait_interpolation(goal_time)

        rospy.loginfo('Return to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Finish example')
        rospy.spin()
