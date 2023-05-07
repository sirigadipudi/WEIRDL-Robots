#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('arm_obstacle')
    wait_for_time()
    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.removeCollisionObject('bottom')
    table_size_x = 0.3
    table_size_y = 2
    table_size_z = 1.23
    table_x = 0.99
    table_y = 0.13
    table_z = 0.59
    planning_scene.addBox('bottom', table_size_x, table_size_y, table_size_z,
	                  table_x, table_y, table_z)
    # Create divider obstacle
    planning_scene.removeCollisionObject('rdivider')
    size_x = 0.3 
    size_y = 0.9
    size_z = 0.4
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = table_y - 0.52
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('rdivider', size_x, size_y, size_z, x, y, z)

    planning_scene.removeCollisionObject('ldivider')
    size_x = 0.3 
    size_y = 0.63
    size_z = 0.4
    x = table_x - (table_size_x / 2) + (size_x / 2)
    y = table_y + 0.53
    z = table_z + (table_size_z / 2) + (size_z / 2)
    planning_scene.addBox('ldivider', size_x, size_y, size_z, x, y, z)

    planning_scene.removeCollisionObject('top')

    planning_scene.addBox('top', table_size_x, table_size_y, table_size_z,
	                  table_x, table_y, table_z + 1.44)
    
    planning_scene.removeCollisionObject('boxr')
    box_l = 0.65
    box_w = 0.08
    box_h = 0.3
    box_x = 0.23
    box_y = -0.671
    box_z = box_h/2
    planning_scene.addBox('boxr', box_l, box_w, box_h,
	                  box_x, box_y, box_z)
    planning_scene.removeCollisionObject('boxl')
    planning_scene.addBox('boxl', box_l, box_w, box_h,
	                  box_x, box_y+0.38, box_z)
    planning_scene.removeCollisionObject('boxt')
    box_l = 0.08
    box_w = 0.38
    box_h = 0.3
    box_x = 0.5
    box_y = -0.469
    box_z = box_h/2
    planning_scene.addBox('boxt', box_l, box_w, box_h,
	                  box_x, box_y, box_z)
    planning_scene.removeCollisionObject('boxb')
    planning_scene.addBox('boxb', box_l, box_w, box_h,
	                  box_x-0.55, box_y, box_z)

if __name__ == '__main__':
    main()
