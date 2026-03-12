#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation as R
def offset_pose_local(pose_world, x=0.0, y=0.0, z=0.0):
    pos = np.array([
        pose_world.pose.position.x,
        pose_world.pose.position.y,
        pose_world.pose.position.z
    ])

    quat = [
        pose_world.pose.orientation.x,
        pose_world.pose.orientation.y,
        pose_world.pose.orientation.z,
        pose_world.pose.orientation.w
    ]

    rot = R.from_quat(quat)

    local_offset = np.array([x, y, z])
    world_offset = rot.apply(local_offset)

    new_pos = pos + world_offset

    pose_world.pose.position.x = float(new_pos[0])
    pose_world.pose.position.y = float(new_pos[1])
    pose_world.pose.position.z = float(new_pos[2])

    return pose_world
class ArucoArmMove(object):

    def __init__(self):
        super(ArucoArmMove, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("aruco_arm_move", anonymous=True)

        robot = moveit_commander.RobotCommander()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.aruco_pose = None

        group_name = "feetech_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.group_names = group_names
        self.aruco_pose = None

        # subscriber
        rospy.Subscriber(
            "/aruco_pose",
            geometry_msgs.msg.PoseStamped,
            self.aruco_callback
        )

        rospy.loginfo("ArucoArmMove node ready")

    def aruco_callback(self, msg):

        rospy.loginfo("Received ArUco pose")

        self.aruco_pose = msg

        self.go_to_pose_goal()

    def go_to_pose_goal(self):
        self.aruco_pose.pose.position.z-=0.075
        self.aruco_pose.pose.position.y-=0.075
        if self.aruco_pose is None:
            return

        try:
            pose_world = self.tf_buffer.transform(
                self.aruco_pose,
                "world",
                rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn("Transform failed: %s", e)
            return

        # approach offset
        pose_world=offset_pose_local(pose_world,0,0,0)

        self.move_group.set_start_state_to_current_state()

        rospy.loginfo("Planning to pose: %s", pose_world)

        self.move_group.set_pose_target(pose_world)

        success = self.move_group.go(wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            rospy.loginfo("Motion executed")
            waypoints = []

            wpose = self.move_group.get_current_pose().pose
            wpose.position.x += 0.03
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.z += 0.05
            waypoints.append(copy.deepcopy(wpose))
            # We want the Cartesian path to be interpolated at a resolution of 1 cm
            # which is why we will specify 0.01 as the eef_step in Cartesian
            # translation.  We will disable the jump threshold by setting it to 0.0,
            # ignoring the check for infeasible jumps in joint space, which is sufficient
            # for this tutorial.
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.01  # waypoints to follow  # eef_step
            )
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -0.088
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = -1.608
            joint_goal[6] = 0.055

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -1.250
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = -1.608
            joint_goal[6] = 0.055

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -1.250
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = 0
            joint_goal[6] = 0.055

            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
        else:
            rospy.logwarn("Motion failed")

        
        

    

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01  # waypoints to follow  # eef_step
        )

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


def main():

    ArucoArmMove()

    rospy.spin()


if __name__ == "__main__":
    main()