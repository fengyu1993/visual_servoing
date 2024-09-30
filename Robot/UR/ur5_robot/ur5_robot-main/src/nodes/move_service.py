#!/usr/bin/env python3
import rospy
from ur5_robot.robot_control import UR_robot
from ur5_robot.srv import MoveToPose, MoveToPoseResponse

def move_to_pose_service(req):
    global ur_robot  # Use a global reference for simplicity
    # Alternatively, for more detailed output, especially useful in debugging
    ur_robot.get_current_pose()

    rospy.loginfo("Received pose goal:\nPosition - x: {0}, y: {1}, z: {2}\nOrientation - x: {3}, y: {4}, z: {5}, w: {6}".format(
        req.pose_goal.position.x, 
        req.pose_goal.position.y, 
        req.pose_goal.position.z, 
        req.pose_goal.orientation.x, 
        req.pose_goal.orientation.y, 
        req.pose_goal.orientation.z, 
        req.pose_goal.orientation.w))

    try:
        fraction = ur_robot.go_to_cartesian_pose(req.pose_goal)
        if fraction < 1.0:
            rospy.logwarn("Only %s%% of the path was planned. Please check if the target pose is reachable." % (fraction * 100))
            # rospy.log("Path planning failed with fraction: %f", fraction)
            return MoveToPoseResponse(False)

        rospy.loginfo("Robot moved to pose successfully.")
        return MoveToPoseResponse(True)

    except Exception as e:
        rospy.logerr("Failed to execute move: %s" % str(e))
        return MoveToPoseResponse(False)

def main():
    global ur_robot

    # Instantiate the UR_robot class
    ur_robot = UR_robot()

    # Initialize ROS service server
    service = rospy.Service('move_to_pose', MoveToPose, move_to_pose_service)
    rospy.loginfo("Service server ready to take pose goals.")

    # Keep the service open
    rospy.spin()

if __name__ == '__main__':
    main()
