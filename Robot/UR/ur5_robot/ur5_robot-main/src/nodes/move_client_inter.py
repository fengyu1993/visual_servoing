#!/usr/bin/env python3
import tkinter as tk
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from ur5_robot.srv import MoveToPose, MoveToPoseRequest

class RobotControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Robot Control Panel")

        # Initialize ROS node
        rospy.init_node('move_robot_client', anonymous=True)
        
        # Pose to modify and send
        self.pose = Pose()
        self.pose.position = Point(0.34, 0.20, 0.4)
        self.pose.orientation = Quaternion(-0.707, 0.003, 0.002, 0.707)

        # Labels for position
        self.labels = {}
        self.entries = {}
        self.variables = {}
        for axis in ['x', 'y', 'z']:
            frame = tk.Frame(master)
            frame.pack()

            label = tk.Label(frame, text=f"{axis.upper()} Position:")
            label.pack(side=tk.LEFT)

            var = tk.DoubleVar()
            var.set(getattr(self.pose.position, axis))
            self.variables[axis] = var

            entry = tk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT)
            self.entries[axis] = entry

            button_plus = tk.Button(frame, text="+", command=lambda a=axis: self.adjust_position(a, 0.01))
            button_plus.pack(side=tk.LEFT)

            button_minus = tk.Button(frame, text="-", command=lambda a=axis: self.adjust_position(a, -0.01))
            button_minus.pack(side=tk.LEFT)

    def adjust_position(self, axis, delta):
        current_val = self.variables[axis].get()
        new_val = current_val + delta
        self.variables[axis].set(new_val)
        setattr(self.pose.position, axis, new_val)
        self.move_robot_to_pose(self.pose)

    def move_robot_to_pose(self, pose):
        rospy.wait_for_service('move_to_pose')
        try:
            move_to_pose = rospy.ServiceProxy('move_to_pose', MoveToPose)
            resp = move_to_pose(pose)
            print("Robot move status:", "Success" if resp.success else "Failed")
        except rospy.ServiceException as e:
            print("Service call failed:", e)

if __name__ == "__main__":
    root = tk.Tk()
    gui = RobotControlGUI(root)
    root.mainloop()

# the failed move may be caused by unreachable position and quaternion parameters