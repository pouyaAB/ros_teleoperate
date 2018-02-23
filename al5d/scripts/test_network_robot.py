import sys
from local_config import config

locals().update(config)

sys.path.append("../../")

import os
from cv_bridge import CvBridge, CvBridgeError
import sys
import std_msgs.msg
from sensor_msgs.msg import Image
import rospy
import threading
import signal
import cv2
import numpy as np
import configparser
import csv
import shutil
import time
from PIL import Image as myImage
import scipy.misc

from std_msgs.msg import Float32MultiArray
from ros_teleoperate.msg import al5d_state

global_config = configparser.ConfigParser()
global_config.read('../../conf.ini')



def signal_handler(signal, frame):
    global TNR
    TNR.end_thread = True
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class TestNetworkRobot(object):
    def __init__(self, image_size, path, robot_command_file, cameras_topic, cache_size=40, channel_num=3, cameras_switch=[False, True, False],
                 camera_num=3):
        rospy.init_node('test_network')
        self.image_size = image_size
        self.task = config['task']
        self.record_path = path
        self.robot_command_file = robot_command_file
        self.last_cameras_time = [rospy.get_time()] * camera_num
        self.last_cameras_printed_ts = [0] * camera_num
        self.last_robot_printed_ts = 0
        self.robot_execution_delay = 3
        self.last_robot_time = rospy.get_time()
        self.cache_size = cache_size
        self.end_thread = False
        self.channel_num = channel_num
        self.cameras_topic = cameras_topic
        self.cameras_switch = cameras_switch
        self.camera_num = camera_num
        self.last_command = []
        self.command_msg = Float32MultiArray()
        self.leap_al5d_msg = [None] * self.cache_size
        self.last_cameras_ts = np.empty((camera_num, self.cache_size))
        self.al5d_cache_index = 0
        self.new_image_captured = False
        self.cameras_images = np.empty(
            (self.camera_num, self.cache_size, self.image_size, self.image_size, self.channel_num))
        self.joints = np.empty((self.cache_size, 8))

        for i in range(self.cache_size):
            self.joints[i] = [0.0, 1.0, 0.7670000195503235, 0.7925000190734863, 0.6265000104904175, 0.4659999907016754, 0.6579999923706055, 0.0]

        self.last_robot_ts = [0] * self.cache_size
        self.images_cache_index = [0] * self.camera_num
        self.all_caches_filled = [False] * self.camera_num
        self.last_command_file_modified = 0
        self.bridge = CvBridge()
        self.pause = False

        for i, isEnable in enumerate(self.cameras_switch):
            if isEnable:
                self.create_folders(os.path.join(self.record_path, 'camera-' + str(i) + '/'))
                rospy.Subscriber(self.cameras_topic[i], Image, self.cameras_callback, callback_args=i)

        self.command_publisher = rospy.Publisher('/robot_command', Float32MultiArray, queue_size=100)
        rospy.Subscriber("/leap_al5d_info", al5d_state, self.leap_al5d_callback)

        # self.thread_images = threading.Thread(target=self.update_images)
        # self.thread_images.start()

        self.thread_commands = threading.Thread(target=self.update_robot_command)
        self.thread_commands.start()

        time.sleep(3)

    def get_next_batch(self, batch_size, camera_id, flip=True):
        if np.all(self.all_caches_filled[camera_id]):
            images_t = self.cameras_images[camera_id][::-1]
            camera_ts = [str(int(x)) for x in self.last_cameras_ts[camera_id][::-1]]
            joint_ts = [str(int(x)) for x in self.last_robot_ts[::-1]]

            # print joint_ts
            # print camera_ts

            next_batch_images = np.empty((batch_size, self.image_size, self.image_size, self.channel_num))
            next_batch_joints = np.empty((batch_size, 7))

            next_batch_images[:] = self.cameras_images[camera_id][-batch_size:]
            # print self.joints[-4:]
            next_batch_joints[:] = self.joints[-batch_size:, 1:]
            # batch_index = 0
            # image_index = 0
            # print joint_ts
            # print camera_ts
            # while batch_index < batch_size:
            #     index = self.find_element(joint_ts, camera_ts[image_index])
            #     if index != None:
            #         # print camera_ts[image_index], 'found'
            #         # print index
            #         next_batch_images[batch_index] = images_t[image_index]
            #
            #         js = self.joints[::-1]
            #         next_batch_joints[batch_index] = js[index][1:]
            #
            #         batch_index += 1
            #         image_index += 1
            #     else:
            #         # print camera_ts[image_index], 'not found'
            #         continue
                # for i, el in enumerate(self.cameras_images[camera_id]):
                #     # moving axis to use plt: i.e [4,100,100] to [100,100,4]
                #     img = self.cameras_images[camera_id][i]
                #     img = img.astype(np.uint8)
                #     print img.dtype, np.max(img), np.min(img), np.shape(img)
                #     img = myImage.fromarray(img, "RGB")
                #     img.show()
                #     raw_input()
            next_batch_images = np.asarray(next_batch_images, dtype=np.float32)
            if flip:
                next_batch_images = np.flip(next_batch_images, axis=2)
            next_batch_images = next_batch_images / 127.5 - 1
            return next_batch_images, next_batch_joints
        else:
            return None, None

    def find_element(self, arr, element):
        for i, a in enumerate(arr):
            if int(a) > int(element):
                continue
            elif a == element:
                return i
            else:
                return None

    def cameras_callback(self, msg, i):
        if not (rospy.get_time() - self.last_cameras_time[i] < 0.08) and not self.pause:
            self.last_cameras_time[i] = rospy.get_time()
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            img = np.array(img, dtype=np.float)
            img = cv2.resize(img, (self.image_size, self.image_size))
            if self.images_cache_index[i] < self.cache_size:
                self.cameras_images[i][self.images_cache_index[i]] = img
                self.last_cameras_ts[i][self.images_cache_index[i]] = str(int(str(msg.header.stamp)[0:-8]))
                self.images_cache_index[i] += 1
            else:
                self.all_caches_filled[i] = True
                self.cameras_images[i][0] = img
                self.cameras_images = np.roll(self.cameras_images, -1, axis=1)

                self.last_cameras_ts[i][0] = str(int(str(msg.header.stamp)[0:-8]))
                self.last_cameras_ts = np.roll(self.last_cameras_ts, -1, axis=1)

            self.new_image_captured = True
            # self.update_images()
            if np.all(self.all_caches_filled[i]):
                self.pause = True
        # np.save(os.path.join(self.record_path, 'camera-' + str(i) + '/'), img)

    def update_images(self):
        # while not self.end_thread:
        if self.new_image_captured and not self.pause:
            # np.save(os.path.join(self.record_path, 'joints'), self.joints)
            for i, isEnable in enumerate(self.cameras_switch):
                # if isEnable:
                #     np.save(os.path.join(self.record_path, 'camera-' + str(i) + '/images'), self.cameras_images[i])
                #     np.save(os.path.join(self.record_path, 'camera-' + str(i) + '/ts'), self.last_cameras_ts[i])
                # self.write_to_file()
                for j in range(self.cache_size):
                    self.save_image(self.cameras_images[i][j], i, j)
                if np.all(self.all_caches_filled):
                    self.new_image_captured = False

    def leap_al5d_callback(self, msg):
        if not (rospy.get_time() - self.last_robot_time < 0.08):
            self.last_robot_time = rospy.get_time()
            if self.al5d_cache_index < self.cache_size:
                ts = str(int(str(msg.header.stamp)[:-8]) + self.robot_execution_delay)
                self.last_robot_ts[self.al5d_cache_index] = ts
                self.leap_al5d_msg[self.al5d_cache_index] = msg.data
                self.joints[self.al5d_cache_index] = [ts] + [x for x in msg.data.data]
                self.al5d_cache_index += 1
            else:
                ts = str(int(str(msg.header.stamp)[:-8]) + self.robot_execution_delay)
                self.leap_al5d_msg = np.roll(self.leap_al5d_msg, -1, axis=0)
                self.leap_al5d_msg[-1] = msg.data

                self.last_robot_ts = np.roll(self.last_robot_ts, -1, axis=0)
                self.last_robot_ts[-1] = ts

                self.joints = np.roll(self.joints, -1, axis=0)
                self.joints[-1] = [ts] + [x for x in msg.data.data]
                # raw_input()

                # print [str(x) for x in self.joints[:, 0]], ts
                # raw_input()

    def save_image(self, img_arr, camera_id, image_name):
        cv2.imwrite(os.path.join(self.record_path, 'camera-' + str(camera_id) + '/' + str(image_name) + '.jpg'),
                    img_arr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])

    def write_to_file(self):
        if not (self.al5d_cache_index < self.cache_size):
            with open(os.path.join(self.record_path, 'joints.txt'), 'w+') as f:
                str_to_append = ""
                for i in range(self.cache_size):
                    str_to_append +=str(self.last_robot_ts[i]) + ',' + str(self.task) + ',' + str(2) + ','
                    data = [x for x in self.leap_al5d_msg[i].data]
                    self.last_robot_printed_ts = self.last_robot_ts[i]

                    sys.stdout.write('\rTimestep: ' + str(self.last_robot_ts[i]))
                    sys.stdout.flush()
                    str_to_append = str_to_append + ','.join(str(e) for e in data) + '\n'

                f.write(str_to_append)

    def update_robot_command(self):
        while not self.end_thread:
            if os.path.exists(self.robot_command_file):
                last_modified = os.stat(self.robot_command_file)[8]
                self.read_command()
                if last_modified > self.last_command_file_modified:
                    time.sleep(1)
                    self.pause = False
                    self.last_command_file_modified = os.stat(self.robot_command_file)[8]

    def read_command(self):
        with open(self.robot_command_file, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in reader:
                # print row
                com = np.asarray(row[:-1], dtype=np.float32)
                com = np.roll(com, -1, axis=0)
                self.command_msg.data = com
                self.command_publisher.publish(self.command_msg)
                self.last_command = row
                # print self.command_msg
                break

    def create_folders(self, foldername):
        if not os.path.exists(foldername):
            os.makedirs(foldername)

    def delete_folder_content(self, folder):
        for the_file in os.listdir(folder):
            file_path = os.path.join(folder, the_file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                # elif os.path.isdir(file_path): shutil.rmtree(file_path)
            except Exception as e:
                print(e)


if __name__ == '__main__':
    global TNR
    TNR = TestNetworkRobot(config['image_size'], config['record_path'], config['robot_command_file'], config['camera_topics'], cache_size=10, cameras_switch=[False, True, False])

    # time.sleep(4)
    # images, joints = TNR.get_next_batch(4, 1)
    # print(np.shape(images), np.shape(joints))
    rospy.spin()
