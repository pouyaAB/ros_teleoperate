#!/usr/bin/env python

import rospy

import sys

sys.path.append("../../")
import configparser
import numpy as np
import time
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

curr_gripper_pos = 0
initialized = False

config = configparser.ConfigParser()
config.read('../../conf.ini')

PI = 3.14159265359
jointMin = [-150.0, -90.0, -40.0, -130.0]
jointMax = [150.0, 90.0, 140.0, 130.0]
jointCommandMask = [0, 0, 0, 0]
Override = 40

def signal_handler(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def send_command(command):
    to_send = std_msgs.msg.String()
    to_send.data = command
    command_topic.publish(to_send)


def joint_state(msg):
    for i in range(4):
        max_exceed = msg.position[i] > ((jointMax[i] - 2) * PI / 180)
        min_exceed = msg.position[i] < ((jointMin[i] + 2) * PI / 180)

        if jointCommandMask[i] == 0 and (max_exceed or min_exceed):
            print 'joint has been reached its limit. Reset and Enable the robot please.'

        if max_exceed:
            jointCommandMask[i] = 1
        elif min_exceed:
            jointCommandMask[i] = -1
        else:
            jointCommandMask[i] = 0


def cartesian_velocity_client(linear, angular):
    """Send a cartesian goal to the action server."""
    # send_command('Reset')

    linear_mask = np.sign(linear) * np.sign(jointCommandMask[0:3])
    linear_mask = np.sign(1 - linear_mask)
    # print str(linear) + ':::' + str(jointCommandMask)
    # print linear_mask
    angular_mask = np.sign(angular[0]) * np.sign(jointCommandMask[3])
    angular_mask = 1 - angular_mask

    vel_goal = sensor_msgs.msg.JointState()
    vel_goal.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
    vel_goal.velocity = [linear[0] * linear_mask[0] * 100, linear[1] * linear_mask[1] * 100,
                         linear[2] * linear_mask[2] * 100, angular[0] * angular_mask * 100, 0, 0]
    vel_goal.name = ['Joint0', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Gripper1', 'Gripper2']
    vel_goal.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    velocity_topic.publish(vel_goal)


def command_received(msg):
    # global pre_pos_command, pre_orientation_command, initialized, curr_robot_finger_pos, x_button_release
    global curr_gripper_pos
    global Override

    pos = controller.get_diff_position(msg)
    orientation = controller.get_orientation(msg)

    # print str(pos) + ":::::" + str(orientation)
    # cartesian_pose_client(pos, [0, 0, 0, 1.0])
    comm = controller.get_button(msg)
    if comm == "gripper-toggle":
        curr_gripper_pos = 1 - curr_gripper_pos
        if curr_gripper_pos == 1:
            send_command('GripperOpen')
        elif curr_gripper_pos == 0:
            send_command('GripperClose')
    elif comm == "triangle":
        print 'Resetting the robot'
        send_command('Reset')
        time.sleep(1)
        print 'Enabling the robot motors'
        send_command('Enable')
    elif comm == "circle":
        print 'Increasing Override'
        Override += 10
        send_command('Override ' + str(Override))
    elif comm == "square":
        print 'Decreasing Override'
        Override -= 10
        send_command('Override ' + str(Override))
    cartesian_velocity_client(pos, orientation)


def status_report(msg):
    if msg.data == 'Not connected':
        send_command('Connect')
    elif 'motorNotEnabled' in msg.data:
        send_command('Reset')
        send_command('Enable')


controller_type = config["controller"]["type"]
rospy.init_node(config["arm"]["type"] + '_controller')
if controller_type == "psmove":
    rospy.Subscriber(config["channels"][controller_type], Float32MultiArray, command_received)
    controller = PSmoveController()  # default controller
if controller_type == "ps4_controller":
    rospy.Subscriber(config["channels"][controller_type], sensor_msgs.msg.Joy, command_received)
    controller = PS4Controller()  # default controller

velocity_topic = rospy.Publisher('/cpr_vel_cartesian',
                                 sensor_msgs.msg.JointState, queue_size=100)

rospy.Subscriber('/CPRMoverErrorCodes', std_msgs.msg.String, status_report)
rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, joint_state)
command_topic = rospy.Publisher('/CPRMoverCommands',
                                std_msgs.msg.String, queue_size=100)


# finger_topic = rospy.Publisher('/' + config["arm"]["type"] + '_arm_driver/fingers/finger_positions',
#                                  jaco_msgs.msg.SetFingersPositionGoal, queue_size=100)


# gripper_client(0)


def main():
    rospy.spin()


if __name__ == '__main__':
    main()
