#!/usr/bin/env python

import rospy
import numpy as np
import sys

sys.path.append("../../")
import configparser
import tf
import actionlib
import signal
import getopt
import jaco_msgs.msg
import geometry_msgs.msg
import kinova_msgs.msg
import sensor_msgs.msg
from kinova_msgs.srv import HomeArm
from std_msgs.msg import Float32MultiArray
from controllers.psmove.psmove_controller import PSmoveController
from controllers.ps4_controller.ps4_controller import PS4Controller

pre_pos_command = [0.0, 0.0, 0.0]
pre_orientation_command = [0.0, 0.0, 0.0, 0.0]

diff_orientation = [0.0, 0.0, 0.0]
pos = [0.0, 0.0, 0.0]
desired_position = np.zeros(3)
desired_orientation = np.zeros(3)
mode = 'velocity'

curr_robot_position = [0.0, 0.0, 0.0]
curr_robot_orientation = [0.0, 0.0, 0.0, 0.0]
curr_robot_finger_pos = 7000
initialized = False
finger_topic = None
velocity_topic = None
config = configparser.ConfigParser()
config.read('../../conf.ini')


def signal_handler(signal, frame):
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def cartesian_velocity_client(linear, angular):
    """Send a cartesian goal to the action server."""
    vel_goal = kinova_msgs.msg.PoseVelocity()
    # vel_goal.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
    vel_goal.twist_linear_x = linear[0] * 10
    vel_goal.twist_linear_y = linear[1] * 10
    vel_goal.twist_linear_z = linear[2] * 10

    vel_goal.twist_angular_x = angular[0] * 10
    vel_goal.twist_angular_y = angular[1] * 10
    vel_goal.twist_angular_z = angular[2] * 10
    rate = rospy.Rate(100)
    velocity_topic.publish(vel_goal)
    rate.sleep()


def cartesian_position_callback(msg):
    global curr_robot_position, curr_robot_orientation, pos, diff_orientation

    curr_robot_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    curr_robot_orientation = [msg.pose.orientation.x, msg.pose.orientation.y,
                              msg.pose.orientation.z, msg.pose.orientation.w]
    # print 'Current Cartesian Position: ' + str(curr_robot_position)
    if mode == 'position':
        curr_robot_euler = tf.transformations.euler_from_quaternion(curr_robot_orientation)
        print curr_robot_euler
        pos_diff = np.asarray([a - b for (a, b) in zip(desired_position, curr_robot_position)])
        ori_diff = np.asarray([a - b for (a, b) in zip(desired_orientation, curr_robot_euler)])
        # print 'position diff' + str(pos_diff)
        pos_diff[np.asarray(map(abs, pos_diff)) < 0.02] = 0
        ori_diff[np.asarray(map(abs, ori_diff)) < 0.2] = 0

        if sum(np.asarray(map(abs, pos_diff)) > 0.02) + sum(np.asarray(map(abs, ori_diff)) > 0.2) > 0:
            print 'sending command to the robot: ' + str(pos_diff/5) + ' ' + str(ori_diff/5)
            pos = pos_diff / 5
            diff_orientation = ori_dif / 5
            # cartesian_velocity_client(pos_diff * 10, [0.0, 0.0, 0.0, 0.0])


def gripper_client(finger_position):
    """Send a gripper goal to the action server."""
    action_address = '/' + config["arm"]["type"] + '_driver/fingers_action/finger_positions'
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


# def cartesian_pose_client(position, orientation):
#     """Send a cartesian goal to the action server."""
#     action_address = '/' + config["arm"]["type"] + '_arm_driver/arm_pose/arm_pose'
#     client = actionlib.SimpleActionClient(action_address, jaco_msgs.msg.ArmPoseAction)
#     client.wait_for_server()
#
#     goal = jaco_msgs.msg.ArmPoseGoal()
#     goal.pose.header = std_msgs.msg.Header(frame_id=(config["arm"]["type"] + '_api_origin'))
#     goal.pose.pose.position = geometry_msgs.msg.Point(
#         x=position[0], y=position[1], z=position[2])
#     goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
#         x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
#
#     client.send_goal(goal)
#
#     if client.wait_for_result(rospy.Duration(10.0)):
#         return client.get_result()
#     else:
#         client.cancel_all_goals()
#         print('the cartesian action timed-out')
#         return None


def command_received(msg):
    # global pre_pos_command, pre_orientation_command, initialized, curr_robot_finger_pos, x_button_release
    global curr_robot_finger_pos, diff_orientation, pos
    diff_orientation = controller.get_orientation(msg)
    # print controller.get_diff_position(msg)
    pos = controller.get_diff_position(msg)

    # cartesian_pose_client(pos, [0, 0, 0, 1.0])
    button = controller.get_button(msg)
    if button == "gripper-toggle":
        curr_robot_finger_pos = 7000 - curr_robot_finger_pos
        gripper_client(curr_robot_finger_pos)


def position_control_init():
    global velocity_topic
    rospy.init_node(config["arm"]["type"] + '_controller')
    rospy.Subscriber('/' + config["arm"]["type"] + '_driver/out/tool_pose',
                     geometry_msgs.msg.PoseStamped, cartesian_position_callback)
    velocity_topic = rospy.Publisher('/' + config["arm"]["type"] + '_driver/in/cartesian_velocity',
                                     kinova_msgs.msg.PoseVelocity, queue_size=1)


def velocity_control_init():
    global controller, velocity_topic, finger_topic
    controller_type = config["controller"]["type"]
    rospy.init_node(config["arm"]["type"] + '_controller')
    if controller_type == "psmove":
        rospy.Subscriber(config["channels"][controller_type], Float32MultiArray, command_received)
        controller = PSmoveController()  # default controller
    if controller_type == "ps4_controller":
        rospy.Subscriber(config["channels"][controller_type], sensor_msgs.msg.Joy, command_received)
        controller = PS4Controller()  # default controller

    velocity_topic = rospy.Publisher('/' + config["arm"]["type"] + '_driver/in/cartesian_velocity',
                                     kinova_msgs.msg.PoseVelocity, queue_size=1)
    finger_topic = rospy.Publisher('/' + config["arm"]["type"] + '_driver/fingers_action/finger_positions',
                                   jaco_msgs.msg.SetFingersPositionGoal, queue_size=100)

    gripper_client(0)
    while True:
        cartesian_velocity_client(pos, diff_orientation)


def main():
    global desired_position, desired_orientation, mode
    optlist, args = getopt.getopt(sys.argv[1:], 'p', ['position'])

    if ('--position', '') in optlist or ('-p', '') in optlist:
        mode = 'position'
        position_control_init()
        desired_position = np.zeros(3)
        desired_position = [0.07, -0.18, 0.3]
        desired_orientation = [-2.77, -1.19, -2.69]
        # desired_position[0] = raw_input("Please enter the desired X cartesian position:")
        # desired_position[1] = raw_input("Please enter the desired Y cartesian position:")
        # desired_position[2] = raw_input("Please enter the desired Z cartesian position:")
        print 'This is the desired cartesian location' + str(desired_position)

        while True:
            cartesian_velocity_client(pos, diff_orientation)
    else:
        velocity_control_init()

    rospy.spin()


if __name__ == '__main__':
    main()
