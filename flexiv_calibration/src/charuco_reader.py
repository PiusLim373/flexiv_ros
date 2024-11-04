#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from flexiv_calibration.srv import GetChArUcoPoseResponse, GetChArUcoPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import copy
import time
from icecream import ic
from tf.transformations import euler_from_matrix, euler_matrix, quaternion_from_matrix

ic.configureOutput(includeContext=True)

FORCE_RP_ZERO = True


class ChArUcoReader:

    def __init__(self):
        self.image_publisher = rospy.Publisher("charuco_reader_output", Image, queue_size=10)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.intrinsics = None
        self.image = None
        self.depth_image = None
        self.pointcloud = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.charuco_board = cv2.aruco.CharucoBoard((9, 5), 0.03, 0.022, self.aruco_dict)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Service("get_charuco_pose", GetChArUcoPose, self.get_charuco_pose_cb)
        self.camera_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.camera_info_cb, queue_size=10
        )
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=10)

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        rospy.loginfo("Camera info received and subscription destroyed.")
        self.camera_info_sub.unregister()

    def image_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv_image = copy.deepcopy(self.image)
        corners, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict)
        if ids is not None:
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, self.image, self.charuco_board
            )
            if charuco_corners is not None and charuco_ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self.charuco_board,
                    self.camera_matrix,
                    self.dist_coeffs,
                    np.empty(1),
                    np.empty(1),
                )

                if success:
                    charuco_frame_offset = np.array(
                        [
                            self.charuco_board.getChessboardSize()[0] * self.charuco_board.getSquareLength() / 2,
                            self.charuco_board.getChessboardSize()[1] * self.charuco_board.getSquareLength() / 2,
                            0,
                        ]
                    )
                    rmat, _ = cv2.Rodrigues(rvec)
                    tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03, 2)

                    # Compute the centroid of the ChArUco corners
                    origin_point = np.float32([[0, 0, 0]])
                    image_points, _ = cv2.projectPoints(origin_point, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                    (u, v) = image_points[0].ravel()
                    center_x = int(u)
                    center_y = int(v)

                    # Draw the centroid on the image
                    cv2.circle(cv_image, (int(center_x), int(center_y)), 3, (255, 255, 0), -1)
                    cv2.putText(
                        cv_image,
                        f"Charuco: ({int(center_x)}, {int(center_y)})",
                        (int(center_x) + 10, int(center_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

        charuco_reader_output = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        charuco_reader_output.header.frame_id = "camera_color_optical_frame"
        charuco_reader_output.header.stamp = rospy.Time.now()
        self.image_publisher.publish(charuco_reader_output)

    def get_charuco_pose_cb(self, req):
        # read the camera image, find the point of the specified charuco id
        rospy.loginfo("Got a request to get charuco point2")
        res = GetChArUcoPoseResponse()
        try:
            rvecs = []
            tvecs = []
            valid_samples = 0

            for i in range(10):
                frame = copy.deepcopy(self.image)
                cv_image = copy.deepcopy(frame)
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)

                if ids is not None:
                    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        corners, ids, frame, self.charuco_board
                    )
                    if charuco_corners is not None and charuco_ids is not None:
                        cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                        success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                            charuco_corners,
                            charuco_ids,
                            self.charuco_board,
                            self.camera_matrix,
                            self.dist_coeffs,
                            np.empty(1),
                            np.empty(1),
                        )

                        if success:
                            charuco_frame_offset = np.array(
                                [
                                    self.charuco_board.getChessboardSize()[0]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    self.charuco_board.getChessboardSize()[1]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    0,
                                ]
                            )

                            rvecs.append(rvec)
                            rmat, _ = cv2.Rodrigues(rvec)
                            tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                            tvecs.append(tvec)
                            valid_samples += 1
                time.sleep(0.05)

            if valid_samples > 5:
                avg_rvec = np.mean(np.array(rvecs), axis=0)
                avg_tvec = np.mean(np.array(tvecs), axis=0)

                rmat, _ = cv2.Rodrigues(avg_rvec)
                mat = np.eye(4)
                mat[:3, :3] = rmat
                mat[:3, 3] = avg_tvec.flatten()

                quaternion = quaternion_from_matrix(mat)
                ic(quaternion)
                translation = avg_tvec.flatten()
                self.publish_transform(translation, quaternion, f"Charuco_intrinsic")
                res.pose.position.x = translation[0]
                res.pose.position.y = translation[1]
                res.pose.position.z = translation[2]
                res.pose.orientation.x = quaternion[0]
                res.pose.orientation.y = quaternion[1]
                res.pose.orientation.z = quaternion[2]
                res.pose.orientation.w = quaternion[3]
                ic(quaternion)
                res.success = True

            else:
                rospy.logwarn(f"Unable to find a valid marker for more than 5 attempts for ID: {req.charuco_id}")
                res.success = False

            return res

        except Exception as e:
            rospy.logerr(f"Error: {str(e)}")
            res.success = False
            return res

    def rotation_matrix_to_rpy(self, R):
        # Extract roll, pitch, yaw from the rotation matrix
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)
        return (roll, pitch, yaw)

    def rotation_matrix_to_quaternion(self, R):
        # Convert a rotation matrix to a quaternion
        q = np.empty((4,), dtype=np.float64)
        t = np.trace(R)
        if t > 0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q

    def publish_transform(self, translation, quad, charuco_id):
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = charuco_id

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = quad[0]
        transform.transform.rotation.y = quad[1]
        transform.transform.rotation.z = quad[2]
        transform.transform.rotation.w = quad[3]

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rospy.init_node("charuco_reader")
    charuco_reader = ChArUcoReader()
    rospy.spin()


def gen_marker():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # aruco_dict.bytesList = aruco_dict.bytesList[4:, :, :]
    board = cv2.aruco.CharucoBoard((4, 4), 0.017, 0.012, aruco_dict)
    board_image = board.generateImage((1000, 1000), None, 10, 1)
    cv2.imshow("charuco", board_image)
    cv2.waitKey(0)


if __name__ == "__main__":
    # gen_marker()
    main()
