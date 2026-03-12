#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf.transformations as tf_trans


class ArucoDetector:

    def __init__(self):

        rospy.init_node("aruco_detector")

        self.bridge = CvBridge()

        self.pose_pub = rospy.Publisher(
            "/aruco_pose",
            PoseStamped,
            queue_size=10
        )

        # RealSense image topic
        rospy.Subscriber(
            "realsense/rgb/image_raw",
            Image,
            self.image_callback
        )

        # RealSense calibration topic
        rospy.Subscriber(
            "realsense/rgb/camera_info",
            CameraInfo,
            self.camera_info_callback
        )

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.parameters
        )

        # marker size in meters
        self.marker_size = 0.05

        # camera parameters (will be filled later)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.prev_tvec = None
        self.publish_threshold = 0.01 # found id, distance changes


    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(msg.K).reshape(3,3)
        self.dist_coeffs = np.array(msg.D)

        rospy.loginfo_once("Camera calibration received")


    def image_callback(self,msg):

        # wait until calibration is received
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)
        

        if ids is not None:

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )
        
            for i in range(len(ids)):
                
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                publish = True

                if self.prev_tvec is not None:

                    dist = np.linalg.norm(tvec - self.prev_tvec)

                    if dist < self.publish_threshold:
                        publish = False

                if publish:

                    pose_msg = PoseStamped()

                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = msg.header.frame_id

                    pose_msg.pose.position.x = tvec[0]
                    pose_msg.pose.position.y = tvec[1]
                    pose_msg.pose.position.z = tvec[2]

                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    #Rz = tf_trans.euler_matrix(0, np.pi/2, 0)[:3, :3] # to rotate pose orentation
                    #rot_matrix = rot_matrix @ Rz
                    transform = np.eye(4)
                    transform[:3,:3] = rot_matrix

                    quat = tf_trans.quaternion_from_matrix(transform)

                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]

                    self.pose_pub.publish(pose_msg)

                    # IMPORTANT: store copy
                    self.prev_tvec = tvec.copy()

                    rospy.loginfo("Published ArUco Pose")
        else:
            # Marker disappeared → reset
            self.prev_tvec = None


if __name__ == "__main__":

    ArucoDetector()
    rospy.spin()