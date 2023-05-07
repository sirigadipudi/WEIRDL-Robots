#! /usr/bin/env python

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point
from pyquaternion import Quaternion
import robot_api
import rospy
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from aruco_msgs.msg import Marker as ARMarker


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise



class ArTagReader(object):
    def __init__(self):
        self.pose = None

    def callback(self, msg):
        self.pose = msg

class PointSReader(object):
    def __init__(self):
        self.pose = None
        self.frame_id = None
        
    def callback(self, msg):
        self.pose = msg.pose
        self.frame_id = msg.header.frame_id

def main():

    rospy.init_node("binToBox")
    wait_for_time()
    
    def average_poses(pose_list):
        num_poses = len(pose_list)
        if num_poses == 0:
            return None
        
        # Initialize the total position and orientation
        total_position = [0.0, 0.0, 0.0]
        total_orientation = [0.0, 0.0, 0.0, 0.0]
        
        # Iterate over the list and compute the total position and orientation
        for pose in pose_list:
            position = pose.position
            orientation = pose.orientation
            
            total_position[0] += position.x
            total_position[1] += position.y
            total_position[2] += position.z
            
            total_orientation[0] += orientation.x
            total_orientation[1] += orientation.y
            total_orientation[2] += orientation.z
            total_orientation[3] += orientation.w
        
        # Compute the average position and orientation
        avg_position = [total_position[0] / num_poses, total_position[1] / num_poses, total_position[2] / num_poses]
        avg_orientation = [total_orientation[0] / num_poses, total_orientation[1] / num_poses, total_orientation[2] / num_poses, total_orientation[3] / num_poses]
        
        # Create a new Pose message with the average position and orientation
        avg_pose = Pose()
        avg_pose.position.x = avg_position[0]
        avg_pose.position.y = avg_position[1]
        avg_pose.position.z = avg_position[2]
        avg_pose.orientation.x = avg_orientation[0]
        avg_pose.orientation.y = avg_orientation[1]
        avg_pose.orientation.z = avg_orientation[2]
        avg_pose.orientation.w = avg_orientation[3]
        
        return avg_pose

    


    gripper = robot_api.Gripper()
    gripper.open()
    torso = robot_api.Torso()
    torso.set_height(0.4)

    head = robot_api.Head()
    head.pan_tilt(0,0)
    
    kwargs = {
        'allowed_planning_time': 10,
        'execution_timeout': 1000,
        'num_planning_attempts': 50,
        'replan': True
    }
    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = robot_api.Arm()
    def shutdown():
       arm.cancel_all_goals()
       rospy.on_shutdown(shutdown)
    error = arm.move_to_pose(start, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    ar_reader = ArTagReader()
    ar_sub = rospy.Subscriber("/aruco_simple/pose", Pose, ar_reader.callback) # Subscribe to AR tag poses, use reader.callback

    poseList = []
    rospy.sleep(0.5)
    for i in range(10):
        while ar_reader.pose == None:
            rospy.loginfo("cant find ar tags!")
            rospy.sleep(0.5)
        pose = ar_reader.pose
        transformed_pose = transform_pose(pose, "camera_color_optical_frame", "camera_depth_optical_frame")
        poseList.append(transformed_pose)
        # print(transformed_pose)
    
    averagePose = average_poses(poseList)
    print(averagePose)
    rospy.set_param("crop_tx", averagePose.position.x)
    rospy.set_param("crop_ty", averagePose.position.y)
    rospy.set_param("crop_tz", averagePose.position.z)
    rospy.set_param("crop_rx", averagePose.orientation.x)
    rospy.set_param("crop_ry", averagePose.orientation.y)
    rospy.set_param("crop_rz", averagePose.orientation.z)
    rospy.set_param("crop_rw", averagePose.orientation.w)
    rospy.set_param('crop_max_x', 0.24)
    rospy.set_param('crop_max_y', 0.2)
    rospy.set_param('crop_max_z', -0.1)
    rospy.set_param('crop_min_x', 0)
    rospy.set_param('crop_min_y', -0.28)
    rospy.set_param('crop_min_z', -0.23)
    rospy.loginfo("set!")
    
    
    reader = PointSReader()
    sub = rospy.Subscriber("/segment_cloud", Marker, reader.callback)
    rospy.sleep(2)

    while reader.pose == None:
        rospy.sleep(0.1)
    
    # q2 = Quaternion(axis=[0, 1, 0], angle=0)
    q3 = Quaternion(axis=[0, 0, 1], angle=0)
    # pose1.position.x = 0.6
    # pose1.position.y = -0.07
    # pose1.position.z = 1.21
    # pose1.orientation = q1*q2*q3
    
    rospy.sleep(2)
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    transformed_pose = transform_pose(reader.pose, "camera_depth_optical_frame", "base_link")
    transformed_pose.position.x -= 0.3
    # transformed_pose.position.z -= 0.05
    ps1.pose = transformed_pose
    ps1.pose.orientation = q3

    rospy.loginfo(ps1)
    # TODO: get the pose to move to
    error = arm.move_to_pose(ps1, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    
    rospy.sleep(1)
    ps2 = PoseStamped()
    ps2.header.frame_id = ps1.header.frame_id
    ps2.pose = ps1.pose
    ps2.pose.position.x += 0.12

    # TODO: get the pose to move to
    error = arm.move_to_pose(ps2)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    
    gripper.close(50)

    ps2.pose.position.z += 0.05
    error = arm.move_to_pose(ps2, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return

    head.pan_tilt(-1.57, 1.57)
    ps2.pose.position.x -= 0.38
    error = arm.move_to_pose(ps2, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    
    
    torso.set_height(0.1)
    rospy.sleep(4)
    ar_reader2 = ArTagReader()
    ar_sub2 = rospy.Subscriber("/aruco_simple/pose2", Pose, ar_reader2.callback) # Subscribe to AR tag poses, use reader.callback
    while ar_reader2.pose == None:
        rospy.loginfo("cant find ar tags!")
        rospy.sleep(0.5)
    pose = ar_reader2.pose
    transformed_pose = transform_pose(pose, "camera_color_optical_frame", "base_link")
    print(transformed_pose)
    rospy.sleep(2)
    transformed_pose.position.x += 0.25
    transformed_pose.position.y -= 0.2
    transformed_pose.position.z += 0.45
    ps1.pose = transformed_pose
    ps1.pose.orientation = Quaternion(-0.5,-0.5,-0.5,0.5)

    rospy.loginfo(ps1)
    # TODO: get the pose to move to
    error = arm.move_to_pose(ps1, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    

    ps1.pose.position.z = 0.5
    error = arm.move_to_pose(ps1, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return

    ps1.pose.position.z = 0.3
    error = arm.move_to_pose(ps1, **kwargs)
    if error is None:
        rospy.loginfo('Moved to point')
    else:
        rospy.logwarn('Failed to move to marker')
        return
    gripper.open()

    
    

    # rospy.sleep(1)
    
    # pose3 = Pose(Point(reader.point.point.x-0.32, reader.point.point.y, reader.point.point.z), Quaternion(0, 0, 0, 0))
    # ps3 = PoseStamped()
    # ps3.header.frame_id = 'base_link'
    # ps3.pose = pose3
    # error = arm.move_to_pose(ps3, **kwargs)
    # if error is None:
    #     rospy.loginfo('Moved to point')
    # else:
    #     rospy.logwarn('Failed to move to marker')
    #     return
    # torso.set_height(0.3)
    # arm.move_to_joints(0.9496175826782226, 0.9272553124084473, -2.231675013182373, 1.661591770480957, 2.770094015499878, 1.98314334241333, -0.13292384827461243)
    # gripper.open()
    # return``

if __name__ == '__main__':
    main()
