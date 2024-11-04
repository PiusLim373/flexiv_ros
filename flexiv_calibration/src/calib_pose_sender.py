#!/usr/bin/python3
import rospy
import sys
import math
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point, PoseStamped, TransformStamped
from std_srvs.srv import Trigger, TriggerResponse
from flexiv_calibration.srv import GetChArUcoPose, GetChArUcoPoseResponse, IntTrigger
import tf2_ros
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from icecream import ic
import tf2_geometry_msgs

from rospkg import RosPack
rospack = RosPack()
flexiv_calibration_path = rospack.get_path("flexiv_calibration")

ic.configureOutput(includeContext=True)
np.set_printoptions(suppress=True)

Z_CHECK = [-0.4]
Y_CHECK = [-20, 0, 20, 30]
X_CHECK = [-30, -20, 0, 20, 30]


class MoveGroupPythonInteface(object):
    def __init__(self):

        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        move_group = moveit_commander.MoveGroupCommander("rizon_arm")
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.calibration_poses = []
        try:
            with open(CALIB_POSE_PATH, "r") as f:
                for line in f:
                    # Remove any leading/trailing whitespace and split the line by commas
                    pose_list = line.strip().split(", ")
                    # Convert each element from string to float
                    pose_list = [float(value) for value in pose_list]
                    # Add the pose list to the calibration_poses list
                    self.calibration_poses.append(pose_list)
            ic(f"Loaded {len(self.calibration_poses)} calibration poses!")
        except FileNotFoundError:
            rospy.logerr(f"File {CALIB_POSE_PATH} not found!")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Service("start_calibration", IntTrigger, self.start_calibration_cb)
        rospy.Service("recorded_joint_calibration", Trigger, self.recorded_joint_calibration_cb)
        rospy.Service("generate_calib_pose", Trigger, self.generate_calib_pose_cb)
        self.get_charuco_pose_srv = rospy.ServiceProxy("get_charuco_pose", GetChArUcoPose)

        rospy.loginfo("Initialization Completed!")

    def move_with_joint(self, joint_goal):
        result = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return result

    def move_with_cartesian(self, pose_list):
        use_curr_quaternion = False
        pose_goal = self.list_to_pose(pose_list)
        print(self.move_group.get_current_pose().pose)
        pose_list = self.generate_waypoints_with_step(
            start_pose=self.move_group.get_current_pose().pose,
            end_pose=pose_goal,
            use_curr_quaternion=use_curr_quaternion,
        )
        for x in pose_list:
            print(x.position.x, x.position.y, x.position.z)
        (path, frac) = self.move_group.compute_cartesian_path(pose_list, 0.01, 0.0)
        print(f"frac: {frac}")
        result = self.move_group.execute(path, wait=True)
        self.move_group.stop()
        return result

    def generate_waypoints_with_step(self, start_pose, end_pose, use_curr_quaternion=False, distance_per_step=0.05):

        step_size = int(
            max(
                [
                    math.ceil(abs(end_pose.position.x - start_pose.position.x) / distance_per_step),
                    math.ceil(abs(end_pose.position.y - start_pose.position.y) / distance_per_step),
                    math.ceil(abs(end_pose.position.z - start_pose.position.z) / distance_per_step),
                ]
            )
        )
        waypoints = []

        incremental = Point(
            (end_pose.position.x - start_pose.position.x) / step_size,
            (end_pose.position.y - start_pose.position.y) / step_size,
            (end_pose.position.z - start_pose.position.z) / step_size,
        )
        print(end_pose)
        for i in range(1, step_size + 1):
            temp_waypoint = Pose()
            temp_waypoint.position.x = start_pose.position.x + i * incremental.x
            temp_waypoint.position.y = start_pose.position.y + i * incremental.y
            temp_waypoint.position.z = start_pose.position.z + i * incremental.z
            if use_curr_quaternion:
                temp_waypoint.orientation.x = start_pose.orientation.x
                temp_waypoint.orientation.y = start_pose.orientation.y
                temp_waypoint.orientation.z = start_pose.orientation.z
                temp_waypoint.orientation.w = start_pose.orientation.w
            else:
                temp_waypoint.orientation.x = end_pose.orientation.x
                temp_waypoint.orientation.y = end_pose.orientation.y
                temp_waypoint.orientation.z = end_pose.orientation.z
                temp_waypoint.orientation.w = end_pose.orientation.w
            waypoints.append(temp_waypoint)

        return waypoints

    def list_to_pose(self, data):
        temp = Pose()
        temp.position.x = data[0]
        temp.position.y = data[1]
        temp.position.z = data[2]
        if len(data) != 7:
            temp.orientation = self.move_group.get_current_pose().pose.orientation
        else:
            temp.orientation.x = data[3]
            temp.orientation.y = data[4]
            temp.orientation.z = data[5]
            temp.orientation.w = data[6]
        return temp

    def pose_to_list(self, pose):
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def generate_calib_pose_cb(self, req):
        rospy.loginfo("Receive request to start auto calibration...")
        res = TriggerResponse()
        get_charuco_pose_res = GetChArUcoPoseResponse()
        get_charuco_pose_res = self.get_charuco_pose_srv.call()
        if get_charuco_pose_res.success:
            charuco_pose = get_charuco_pose_res.pose
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            # listener.waitForTransform("world", "camera_color_optical_frame", rospy.Time(), rospy.Duration(5.0))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "camera_color_optical_frame"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = charuco_pose

            cam_T_firstcharuco = posestamped_to_t_matrix(pose_stamped)
            R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            gripper_based_cam_T_firstcharuco = cam_T_firstcharuco @ R_z_90
            # gripper_based_cam_T_firstcharuco = cam_T_firstcharuco
            gripper_based_charuco_pose = t_matrix_to_posestamped(gripper_based_cam_T_firstcharuco)
            gripper_based_charuco_pose.header.frame_id = "camera_color_optical_frame"

            transform = tf_buffer.lookup_transform(
                "world", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0)
            )
            firstcharuco_in_world = tf2_geometry_msgs.do_transform_pose(gripper_based_charuco_pose, transform)
            # firstcharuco_in_world = listener.transformPose("world", gripper_based_charuco_pose)
            ic(firstcharuco_in_world)

            self.publish_transform(firstcharuco_in_world, "world", "first_charuco")
            self.calibration_poses = []
            for z in Z_CHECK:
                for x_theta in X_CHECK:
                    for y_theta in Y_CHECK:
                        R_x_theta = np.array(
                            [
                                [1, 0, 0, 0],
                                [0, math.cos(math.radians(x_theta)), -math.sin(math.radians(x_theta)), 0],
                                [0, math.sin(math.radians(x_theta)), math.cos(math.radians(x_theta)), 0],
                                [0, 0, 0, 1],
                            ]
                        )
                        R_y_theta = np.array(
                            [
                                [math.cos(math.radians(y_theta)), 0, math.sin(math.radians(y_theta)), 0],
                                [0, 1, 0, 0],
                                [-math.sin(math.radians(y_theta)), 0, math.cos(math.radians(y_theta)), 0],
                                [0, 0, 0, 1],
                            ]
                        )

                        trans = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z], [0, 0, 0, 1]])
                        temp = gripper_based_cam_T_firstcharuco @ R_x_theta @ R_y_theta
                        calib_pose_mat = temp @ trans
                        calib_pose = t_matrix_to_posestamped(calib_pose_mat)
                        calib_pose.header.frame_id = "camera_color_optical_frame"
                        calib_pose_in_world = tf2_geometry_msgs.do_transform_pose(calib_pose, transform)
                        # calib_pose_in_world = listener.transformPose("world", calib_pose)
                        self.publish_transform(calib_pose_in_world, "world", f"{x_theta}_{y_theta}_{z}")
                        self.calibration_poses.append(self.pose_to_list(calib_pose_in_world.pose))
            print(f"{len(self.calibration_poses)} calibration poses generated!")
            with open(CALIB_POSE_PATH, "w") as f:
                for pose_list in self.calibration_poses:
                    pose_str = ", ".join(map(str, pose_list))
                    f.write(f"{pose_str}\n")
            f.close()
            res.success = True
            return res
        else:
            rospy.logerr("Failed to get ChArUco pose!")
            res.success = False

    def publish_transform(self, pose, parent, child):
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child

        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z

        transform.transform.rotation.x = pose.pose.orientation.x
        transform.transform.rotation.y = pose.pose.orientation.y
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.transform.rotation.w = pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

    def start_calibration_cb(self, req):
        if len(self.calibration_poses) == 0:
            rospy.logerr("No calibration poses generated!")
            return TriggerResponse(success=False)
        rospy.loginfo(f"Starting Calibration from index {req.starting_index}...")
        for index, pose_list in enumerate(self.calibration_poses):
            if index < req.starting_index:
                rospy.loginfo(f"Skipping pose of index {index}")
                continue
            if self.move_with_cartesian(pose_list):
                input("Calibration pose reached! Please capture the image and press enter to continue...")
            else:
                rospy.logerr(f"Error when moving to calibration pose of index {index}")
                return TriggerResponse(success=False)
        rospy.loginfo("Calibration completed!")
        return TriggerResponse(success=True)

    def recorded_joint_calibration_cb(self, req):
        res = TriggerResponse()
        rospy.loginfo("Starting Calibration...")
        for joint in CALIB_JOINT:
            if len(joint) != 7:
                rospy.logerr("7 joint angles are expected!")
                res.success = False
                return res
            if not self.move_with_joint(req.joint_angles):
                rospy.logerr(f"Error when moving to joint: {joint}")
                res.success = False
                return res
            input("Press Enter to continue...")
        rospy.loginfo("Calibration completed!")
        res.success = True
        return res


def posestamped_to_t_matrix(posestamped):
    # Extract position
    position = posestamped.pose.position
    translation = np.array([position.x, position.y, position.z])

    # Extract orientation
    orientation = posestamped.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

    # Convert quaternion to a rotation matrix
    rotation_matrix = quaternion_matrix(quaternion)

    # Create the transformation matrix
    t_matrix = np.identity(4)
    t_matrix[0:3, 0:3] = rotation_matrix[0:3, 0:3]  # Set rotation part
    t_matrix[0:3, 3] = translation  # Set translation part

    return t_matrix


def t_matrix_to_posestamped(t_matrix):
    quaternion = quaternion_from_matrix(t_matrix)

    # Extract translation
    translation = t_matrix[0:3, 3]
    # Create the PoseStamped message
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = translation[0]
    pose.pose.position.y = translation[1]
    pose.pose.position.z = translation[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose


if __name__ == "__main__":
    rospy.init_node("calib_pose_sender")
    CALIB_JOINT = rospy.get_param("~joint_values", [])
    CALIB_POSE_PATH = rospy.get_param(
        "~calib_pose_path",
        f"{flexiv_calibration_path}/config/calibration_poses.txt",
    )

    ROBOT = MoveGroupPythonInteface()
    rospy.spin()
