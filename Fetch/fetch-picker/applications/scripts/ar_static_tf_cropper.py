#!/usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node("ar_listener")
    
    wait_for_time()
                                                  
    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback
    rospy.sleep(0.5)
    while len(reader.markers) == 0:
        rospy.loginfo("cant find markers!")
        rospy.sleep(0.5)
    while True:
        for marker in reader.markers:
            rospy.set_param("crop_min_x", marker.pose.pose.position.x-0.9)
            rospy.set_param("crop_min_y", marker.pose.pose.position.y+0.1)
            rospy.set_param("crop_min_z", marker.pose.pose.position.z-1.0)
            rospy.set_param("crop_max_x", marker.pose.pose.position.x-0.8)
            rospy.set_param("crop_max_y", marker.pose.pose.position.y+0.22)
            rospy.set_param("crop_max_z", marker.pose.pose.position.z-0.6)
            rospy.loginfo('set!')
            rospy.sleep(0.5)
            
 
if __name__ == '__main__':
    main()
