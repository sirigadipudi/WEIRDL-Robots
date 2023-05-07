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
    while not rospy.is_shutdown():
        # Get the first AR tag marker
        marker = reader.markers[0]
        rospy.set_param("crop_tx", marker.pose.pose.position.x)
        rospy.set_param("crop_ty", marker.pose.pose.position.y)
        rospy.set_param("crop_tz", marker.pose.pose.position.z)
        rospy.set_param("crop_rx", marker.pose.pose.orientation.x)
        rospy.set_param("crop_ry", marker.pose.pose.orientation.y)
        rospy.set_param("crop_rz", marker.pose.pose.orientation.z)
        rospy.set_param("crop_rw", marker.pose.pose.orientation.w)
        rospy.loginfo('set!')
        rospy.sleep(0.5)
            
 
if __name__ == '__main__':
    main()
