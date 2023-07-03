#!/usr/bin/env python3

import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

def active_cb(extra):
    rospy.loginfo("Goal pose being processed")
def feedback_cb(feedback):
    # rospy.loginfo("Current location: "+str(feedback))
    pass
def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

#Initial Node
rospy.init_node('goal_pose')

#Get goal position from ArUco marker
goal_pose = rospy.wait_for_message('/id101/aruco_single/pose', PoseStamped)


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.

    source: https://stackoverflow.com/a/34374437
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def check_goal_reached(init_pose, goal_pose):
    """
        Check if goal is reached

        Args:

        init_pose: position of the turtlebot

        goal_pose: goal position

        Returns:

        True if init_pose close to the goal_pose, otherwise false
        """
    if(init_pose.pose.position.x > goal_pose.pose.position.x - 0.1 and init_pose.pose.position.x < goal_pose.pose.position.x + 0.1\
        and init_pose.pose.position.y > goal_pose.pose.position.y - 0.1 and init_pose.pose.position.y < goal_pose.pose.position.y + 0.1):
        return True
    else:
        return False

#Get initial position of turtlebot
init_msg = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)

#Initialize navigation stack from actionlib
nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
nav_client.wait_for_server()
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

# Since turtlebot has odemetry of x = 0 and y = 0 when booting up, directly get relative goal position from turtlebot
# and multiply by a scale factor
x = (goal_pose.pose.position.x - init_msg.pose.position.x)*1.65
y = -(goal_pose.pose.position.y - init_msg.pose.position.y)*1.2

#Get euler angle between the turtlebot and goal position
orientation_q = init_msg.pose.orientation
orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
(roll, pitch, yaw) = euler_from_quaternion (orientation_list)

# Since turtlebot has orientation 0 after bringup, rotate the goal position in the camera image to fit turtlebot orientation.
dx = goal_pose.pose.position.x - init_msg.pose.position.x
dy = -(goal_pose.pose.position.y - init_msg.pose.position.y)
theta = math.atan2(dy, dx)
print("theta:",theta * 180 / math.pi)
print("yaw: ", yaw * 180 / math.pi)
x_after, y_after = rotate((0,0), (x,y), yaw)

# Set up correct relative goal position.
goal.target_pose.pose.position.x = x_after
goal.target_pose.pose.position.y = y_after
print(goal.target_pose.pose.position.x)
print(goal.target_pose.pose.position.y)

goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.x = 0.0
goal.target_pose.pose.orientation.y = 0.0
goal.target_pose.pose.orientation.z = goal_pose.pose.orientation.y
goal.target_pose.pose.orientation.w = 0.0
init_msg = rospy.wait_for_message('/id100/aruco_single/pose', PoseStamped)

# Activate navigation stack with goal position for tuetlbot to navigate.
nav_client.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = nav_client.wait_for_result()


if not finished:
    rospy.logerr("Action server not available")
else:
    rospy.loginfo(nav_client.get_result())

