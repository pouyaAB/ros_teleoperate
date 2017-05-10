#!/usr/bin/env python

import rospy

import sys
sys.path.append("../../")
import configparser
import tf
import actionlib
import signal
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from controllers.psmove.psmove_controller import PSmoveController
from controllers.ps4_controller.ps4_controller import PS4Controller

pre_pos_command = [0.0, 0.0, 0.0]
pre_orientation_command = [0.0, 0.0, 0.0, 0.0]

curr_robot_position = [0.0, 0.0, 0.0]
curr_robot_orientation = [0.0, 0.0, 0.0, 0.0]
curr_robot_finger_pos = 7000
initialized = False

config = configparser.ConfigParser()
config.read('../../conf.ini')


def signal_handler(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def robot_position_callback(msg):
    global curr_robot_position
    global curr_robot_orientation

    curr_robot_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    curr_robot_orientation = [msg.pose.orientation.x, msg.pose.orientation.y,
                              msg.pose.orientation.z, msg.pose.orientation.w]


def cartesian_velocity_client(linear, angular):
    """Send a cartesian goal to the action server."""

    vel_goal = geometry_msgs.msg.TwistStamped()
    vel_goal.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
    vel_goal.twist.linear = geometry_msgs.msg.Vector3(
        x=linear[0], y=linear[1], z=linear[2])
    vel_goal.twist.angular = geometry_msgs.msg.Vector3(
        x=angular[0], y=angular[1], z=angular[2])

    velocity_topic.publish(vel_goal)


def gripper_client(finger_position):
    """Send a gripper goal to the action server."""
    action_address = '/' + config["arm"]["type"] + '_arm_driver/fingers/finger_positions'
    client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_position)
    goal.fingers.finger2 = float(finger_position)
    goal.fingers.finger3 = float(finger_position)

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('the gripper action timed-out')
        return None


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + config["arm"]["type"] + '_arm_driver/arm_pose/arm_pose'
    client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = jaco_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('the cartesian action timed-out')
        return None


def command_received(msg):
    # global pre_pos_command, pre_orientation_command, initialized, curr_robot_finger_pos, x_button_release
    global curr_robot_finger_pos
    diff_orientation = controller.get_orientation(msg)
    # print controller.get_diff_position(msg)
    pos = controller.get_diff_position(msg)
    cartesian_velocity_client(pos, diff_orientation)
    # cartesian_pose_client(pos, [0, 0, 0, 1.0])
    if controller.get_button(msg) == "gripper-toggle":
        curr_robot_finger_pos = 7000 - curr_robot_finger_pos
        gripper_client(curr_robot_finger_pos)

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
velocity_topic = rospy.Publisher('/' + config["arm"]["type"] + '_arm_driver/in/cartesian_velocity',
                                 geometry_msgs.msg.TwistStamped, queue_size=100)
finger_topic = rospy.Publisher('/' + config["arm"]["type"] + '_arm_driver/fingers/finger_positions',
                                 jaco_msgs.msg.SetFingersPositionGoal, queue_size=100)


gripper_client(0)


def main():
    rospy.spin()


if __name__ == '__main__':
    main()
