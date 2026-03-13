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
        self.aruco_pose.pose.position.z-=0.075 # offseting
        self.aruco_pose.pose.position.y-=0.075
        if self.aruco_pose is None:
            return

        # change frame id to match the robot arm
        try:
            pose_world = self.tf_buffer.transform(
                self.aruco_pose,
                "world",
                rospy.Duration(1.0)
            )
        except Exception as e:
            rospy.logwarn("Transform failed: %s", e)
            return

        
        pose_world=offset_pose_local(pose_world,0,0,0)

        self.move_group.set_start_state_to_current_state()

        rospy.loginfo("Planning to pose: %s", pose_world)

        self.move_group.set_pose_target(pose_world)

        success = self.move_group.go(wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            rospy.loginfo("Motion executed")

            #cartesian movement
            waypoints = []
            
            wpose = self.move_group.get_current_pose().pose
            wpose.position.x += 0.03
            waypoints.append(copy.deepcopy(wpose))
            wpose.position.z += 0.05
            waypoints.append(copy.deepcopy(wpose))
 
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.01  # waypoints to follow  # eef_step
            )
            self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            #manually change each joint, 
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -0.088
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = -1.608
            joint_goal[6] = 0.055
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

            #manually change each joint, rotate arm
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -1.250 # change here to rotate first joint from the base
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = -1.608
            joint_goal[6] = 0.055

            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()

            #manually change each joint, drop
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = -1.250
            joint_goal[1] = 1.515
            joint_goal[2] = 0.125
            joint_goal[3] = -1.135
            joint_goal[4] = -0.018
            joint_goal[5] = 0       # change here to rotate the last 2nd joint to drop item
            joint_goal[6] = 0.055
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
        else:
            rospy.logwarn("Motion failed")


def main():

    ArucoArmMove()

    rospy.spin()


if __name__ == "__main__":
    main()