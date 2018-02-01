#!/usr/bin/env python
import numpy as np
import rospy

import sys

sys.path.append("../../")
import configparser
import tf
import actionlib
import signal
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from controllers.psmove.psmove_controller import PSmoveController
from controllers.ps4_controller.ps4_controller import PS4Controller
#[-13.5040283203125, 69.99999999999994, 330.0, -0.10000000000000001, 0.0, 1, 0, human]
pre_pos_command = [0.0, 0.0, 0.0]
pre_orientation_command = [0.0, 0.0, 0.0, 0.0]
open_gripper = 1.0
SPEED = 20

waiting_robot_position = [-13.5040283203125, 250, 250.0]
waiting_robot_orientation = [-0.10000000000000001, 0.0]

sitting_robot_position = [-13.5040283203125, 50, 250.0]
sitting_robot_orientation = [-0.10000000000000001, 0.0]

robotCharge = True
pause = False
print 'robot in charge: ' + str(robotCharge)

curr_robot_position = [-13.5040283203125, 250, 250.0]
curr_robot_orientation = [-0.10000000000000001, 0.0]
initialized = False


ranges = [  # MIN, CENTER, MAX
            [600, 1500, 2400],# BASE
            [600, 1500, 2200],# SHOULDER
            [600, 1250, 2200],# ELBOW
            [600, 1500, 2400],# WRIST
            [600, 1350, 2400],# WRIST_ROTATE
            [600, 1600, 2400] # GRIPPER
         ]


config = configparser.ConfigParser()
config.read('../../conf.ini')


def signal_handler(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# def robot_position_callback(msg):
#     global curr_robot_position
#     global curr_robot_orientation
#
#     curr_robot_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
#     curr_robot_orientation = [msg.pose.orientation.x, msg.pose.orientation.y,
#                               msg.pose.orientation.z, msg.pose.orientation.w]


# def cartesian_velocity_client(linear, angular):
#     """Send a cartesian goal to the action server."""
#     vel_goal = geometry_msgs.msg.TwistStamped()
#     vel_goal.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
#     vel_goal.twist.linear = geometry_msgs.msg.Vector3(
#         x=linear[0], y=linear[1], z=linear[2])
#     vel_goal.twist.angular = geometry_msgs.msg.Vector3(
#         x=angular[0], y=angular[1], z=angular[2])
#
#     velocity_topic.publish(vel_goal)
def check_ranges():
    curr_robot_position[0] = np.clip([curr_robot_position[0]], ranges[0][0], ranges[0][2])[0]
    curr_robot_position[1] = np.clip([curr_robot_position[1]], ranges[1][0], ranges[1][2])[0]
    curr_robot_position[2] = np.clip([curr_robot_position[2]], ranges[2][0], ranges[2][2])[0]

    curr_robot_orientation[0] = np.clip([curr_robot_orientation[0]], ranges[3][0], ranges[3][2])[0]
    curr_robot_orientation[1] = np.clip([curr_robot_orientation[1]], ranges[4][0], ranges[4][2])[0]


def reset_robot():
    global curr_robot_position, curr_robot_orientation, open_gripper, robotCharge
    global sitting_robot_position, sitting_robot_orientation
    command_message = Float32MultiArray()

    if robotCharge:
        curr_robot_position = waiting_robot_position
        curr_robot_orientation = waiting_robot_orientation
    else:
        curr_robot_position = sitting_robot_position
        curr_robot_orientation = sitting_robot_orientation

    # check_ranges()
    open_gripper = 1
    command_message.data = np.concatenate([curr_robot_position, curr_robot_orientation, [1]])
    # command_message.data = [-13.5040283203125, 250, 250.0, -0.10000000000000001, 0.0, 1, 0, 0]
    al5d_publisher.publish(command_message)


def command_received(msg):
    global curr_robot_position, curr_robot_orientation, open_gripper, robotCharge, pause

    diff_orientation = controller.get_orientation(msg)
    pos = controller.get_diff_position(msg)
    # cartesian_velocity_client(pos, diff_orientation)
    pre_pos_command = curr_robot_position
    pre_orientation_command = curr_robot_orientation

    bt = controller.get_button(msg)
    if bt == "gripper-toggle":
        open_gripper = 1 - open_gripper
    elif bt == "circle":

        if not pause:
            robotCharge = not robotCharge
        if not pause:
            reset_robot()
        pause = not pause
        print 'robot in charge: ' + str(robotCharge)
        return
    elif bt == "triangle":
        robotCharge = not robotCharge
        print 'robot in charge: ' + str(robotCharge)

    command_message = Float32MultiArray()

    curr_robot_position = np.add(curr_robot_position, np.multiply([pos[1], pos[0], pos[2]], SPEED))
    curr_robot_orientation = np.add(curr_robot_orientation, np.multiply(diff_orientation[0:2], 0.1))

    # check_ranges()
    command_message.data = np.concatenate([curr_robot_position, curr_robot_orientation, [open_gripper]])
    # command_message.data = [-13.5040283203125, 250, 250.0, -0.10000000000000001, 0.0, 1, 0, 0]
    if not pause and robotCharge:
        al5d_publisher.publish(command_message)


controller_type = config["controller"]["type"]
rospy.init_node(config["arm"]["type"] + '_controller')
if controller_type == "psmove":
    rospy.Subscriber(config["channels"][controller_type], Float32MultiArray, command_received)
    controller = PSmoveController()  # default controller
if controller_type == "ps4_controller":
    rospy.Subscriber(config["channels"][controller_type], sensor_msgs.msg.Joy, command_received)
    controller = PS4Controller()  # default controller
# rospy.Subscriber('/' + config["arm"]["type"] + '_arm_driver/out/tool_position',
#                  geometry_msgs.msg.PoseStamped, robot_position_callback)
al5d_publisher = rospy.Publisher('/move_info',
                                 Float32MultiArray, queue_size=100)


def main():
    rospy.spin()


if __name__ == '__main__':
    main()
