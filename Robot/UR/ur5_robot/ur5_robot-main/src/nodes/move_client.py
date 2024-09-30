#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from ur5_robot.srv import MoveToPose, MoveToPoseRequest

def move_robot_to_pose(pose):
    rospy.wait_for_service('move_to_pose')
    try:
        move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
        resp = move_to_pose(pose)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def main():
    rospy.init_node('move_robot_client')

    # Example pose
    example_pose = Pose()
    example_pose.position = Point(0.34, 0.20, 0.4)  # Example position in meters
    example_pose.orientation = Quaternion(-0.7070314623250534, 0.00260322079659095,  0.002419391444105879,  0.707173162011272) # x, y, z, w
    result = move_robot_to_pose(example_pose)
    if result:
        print("Robot moved successfully!")
    else:
        print("Failed to move robot.")
if __name__ == "__main__":
    main()
