import numpy as np

XARM_SDK = '/home/xarm/Desktop/xArm-Python-SDK'
import sys

sys.path.append(XARM_SDK)
from xarm.wrapper import XArmAPI

import rospy
from std_msgs.msg import String, Float32MultiArray

# The default home position for the Xarm, when we start up the arm, it will go to the home position
pos = np.array([553.490479, 29.007273, 424.868439])

def callback(data):
    global pos
    pos = np.asarray(data.data)
    
def listener():
    xarm_ip = '192.168.1.220'
    # franka_ip = '127.0.0.1'
    rospy.init_node('listener', anonymous=True)
    r = rospy.Rate(100)
    robot = XArmAPI(xarm_ip)
    robot.connect(port=xarm_ip)
    robot.motion_enable(enable=True)
    robot.set_mode(1)
    robot.set_state(0)
    robot.set_vacuum_gripper(False)
    rospy.Subscriber("commanded_positions", Float32MultiArray, callback)
    min_lim = np.array([-2,-2,-2])
    max_lim = np.array([2, 2, 2])
    # spin() simply keeps python from exiting until this node is stopped
    while True:
        print(pos)
        cur_pos = robot.get_position()[1]
        delta = pos[:3].copy() - cur_pos[:3]
        delta_clipped = np.clip(delta, min_lim, max_lim)
        new_pos = np.asarray(cur_pos)
        new_pos[:3] += delta_clipped
        robot.set_servo_cartesian(new_pos)
        r.sleep()

if __name__ == '__main__':
    listener()