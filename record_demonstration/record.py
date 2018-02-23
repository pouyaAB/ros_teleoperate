import sys
sys.path.append("../")

#! /usr/bin/python

import sensor_msgs.msg
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import os
from std_msgs.msg import Float32MultiArray
from os.path import expanduser
import signal
import threading
from multiprocessing import Pool
import time
from random import randint
from ros_teleoperate.msg import al5d_state
import configparser
from controllers.psmove.psmove_controller import PSmoveController
from controllers.ps4_controller.ps4_controller import PS4Controller

global_config = configparser.ConfigParser()
global_config.read('../conf.ini')

from local_config import config

locals().update(config)


class RecordDemonstrations(object):
    def __init__(self, task, user_id, image_size, path, cameras_topic, cameras_switch=[True, False], camera_num=3):
        rospy.init_node('record_demonstration')
        self.task = task
        self.user_id = user_id
        self.image_shape = (image_size, image_size)
        self.recordDelay = .1
        self.cameras_switch = cameras_switch
        self.path = path

        self.last_cameras_time = [rospy.get_time()] * camera_num
        self.last_cameras_ts = [0] * camera_num
        self.last_robot_time = rospy.get_time()
        self.last_robot_ts = 0
        self.last_robot_printed_ts = 0
        self.last_cameras_printed_ts = [0] * camera_num
        self.robot_execution_delay = 3
        self.demonstration_num = 0

        self.record_human = config['record_human']
        self.inCharge = 'robot'

        self.cameras_topic = cameras_topic
        self.leap_al5d_msg = None
        self.camera_num = camera_num
        self.cameras_msg = [None] * camera_num
        self.task_description = {
            5001: 'Picking Up Something',
            5002: 'Pushing something from left to right',
            5003: 'Moving something closer to something',
            5004: 'Putting something into something',
            5005: 'taking something out of something'
        }

        rospy.Subscriber("/leap_al5d_info", al5d_state, self.leap_al5d_callback)

        controller_type = global_config["controller"]["type"]
        if controller_type == "psmove":
            rospy.Subscriber(global_config["channels"][controller_type], Float32MultiArray, self.controller_callback)
            self.controller = PSmoveController()  # default controller
        if controller_type == "ps4_controller":
            rospy.Subscriber(global_config["channels"][controller_type], sensor_msgs.msg.Joy, self.controller_callback)
            self.controller = PS4Controller()  # default controller

        self.curr_file_path = self.init_record_folder()

        self.bridge = CvBridge()
        self.task_complete_count = 0
        self.rate = rospy.Rate(self.recordDelay * 1000)
        self.last_reward_time = 0
        self.start_time = rospy.get_time()
        self.end_thread = False
        self.pause = False
        self.thread = threading.Thread(target=self.update)
        self.thread.start()

    def init_record_folder(self):
        task_identifier = str(randint(0, 1000000))
        file_path = self.path + str(self.task) + '/' + task_identifier

        while os.path.exists(file_path):
            task_identifier = str(randint(0, 1000000))
            file_path = self.path + str(self.task) + '/' + task_identifier

        self.create_folders(file_path + '/robot')

        if self.record_human:
            self.create_folders(file_path + '/human')

        for i, isEnable in enumerate(self.cameras_switch):
            if isEnable:
                self.create_folders(file_path + '/robot/camera-' + str(i) + '/')
            if self.record_human:
                self.create_folders(file_path + '/human/camera-' + str(i) + '/')
            rospy.Subscriber(self.cameras_topic[i], Image, self.cameras_callback, callback_args=i)

        self.write_file_header(file_path)
        self.demonstration_num += 1
        print str(self.demonstration_num) + ' demonstration folder created: ' + file_path
        return file_path

    def toggle_in_charge(self):
        if self.inCharge == 'robot':
            self.inCharge = 'human'
        elif self.inCharge == 'human':
            self.inCharge = 'robot'

        print '\n' + self.inCharge + ' going to demonstrate next. Press O to start the demonstration'

    def controller_callback(self, msg):
        bt = self.controller.get_button(msg)
        if bt == "circle":
            self.pause = not self.pause
            if self.pause and self.record_human:
                self.toggle_in_charge()

            if self.pause and self.inCharge == 'robot':
                self.curr_file_path = self.init_record_folder()
            return

    def leap_al5d_callback(self, msg):
        self.last_robot_time = rospy.get_time()
        self.leap_al5d_msg = msg.data
        self.last_robot_ts = str(int(str(msg.header.stamp)[:-8]) + self.robot_execution_delay)

    def cameras_callback(self, msg, i):
        # print 'saving image of camera number ' + str(i)
        if not (rospy.get_time() - self.last_cameras_time[i] < 0.08):
            self.last_cameras_time[i] = rospy.get_time()
            self.last_cameras_ts[i] = str(msg.header.stamp)[0:-8]
            self.cameras_msg[i] = msg

    def update(self):
        while not rospy.is_shutdown() and not self.end_thread:
            if not self.pause:
                for i, msg in enumerate(self.cameras_msg):
                    if msg is not None and self.last_cameras_printed_ts[i] != self.last_cameras_ts[i]:
                        self.save_image(msg, i)
                if self.inCharge == 'robot' and self.leap_al5d_msg is not None and self.last_robot_ts != self.last_robot_printed_ts:
                    self.append_to_file()

            self.rate.sleep()

    def write_file_header(self, path):
        with open(path + '.txt', 'w') as f:
            f.write(str(time.strftime('%l:%M%p %z on %b %d, %Y')) + '\n' + str(self.task_description[self.task]) + '\n')
            f.write('timestamp,task,user,gripper,joint1,joint2,joint3,joint4,joint5,joint6')

    def append_to_file(self):
        with open(self.curr_file_path + '.txt', 'a') as f:
            str_to_append = '\n' + str(self.last_robot_ts) + ',' + str(self.task) + ',' + str(self.user_id) + ','
            data = [x for x in self.leap_al5d_msg.data]
            self.last_robot_printed_ts = self.last_robot_ts

            sys.stdout.write('\rTimestep: ' + str(self.last_robot_ts) + ' Task done: ' + str(self.task_complete_count))
            sys.stdout.flush()
            str_to_append = str_to_append + ','.join(str(e) for e in data)
            f.write(str_to_append)

    def save_image(self, img_msg, camera_id):
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            img = np.array(img, dtype=np.float)
        except CvBridgeError, e:
            print(e)
        else:
            # img = img[0:540, 250:840]
            img = cv2.resize(img, self.image_shape)
            cv2.imwrite(self.curr_file_path + '/' + self.inCharge + '/camera-' + str(camera_id) + '/' + str(self.last_cameras_ts[camera_id]) +
             '.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            self.last_cameras_printed_ts[camera_id] = self.last_cameras_ts[camera_id]

    def create_folders(self, foldername):
        if not os.path.exists(foldername):
            try:
                os.makedirs(foldername)
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        else:
            print 'Folder ' + str(foldername) + 'exists! Try again!'

if __name__ == '__main__':
    record_demonstration = RecordDemonstrations(config['task_id'], config['user_id'], config['image_size'],
                                               config['record_path'], config['camera_topics'], config['cameras_switch'])
    rospy.spin()

