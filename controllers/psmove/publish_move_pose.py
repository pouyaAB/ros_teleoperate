#! /usr/bin/python
import configparser
import sys
import tf
import rospy
import signal
from std_msgs.msg import Float32MultiArray

button_pressed = False


def signal_handler(signal, frame):
    sys.exit(0)


def quaternion_to_euler(aa, bb, cc, dd):
    quaternion = (aa, bb, cc, dd)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll, pitch, yaw

signal.signal(signal.SIGINT, signal_handler)

config = configparser.ConfigParser()
config.read('../../conf.ini')

rospy.init_node('psmove')
psmove_topic = rospy.Publisher(config["channels"]["psmove"], Float32MultiArray, queue_size=100)

sys.path.insert(0, config["libraries"]["psmove_build_folder"])
import psmove

tracker = psmove.PSMoveTracker()
move = psmove.PSMove()
move.enable_orientation(True)
# Message send to the ros
psmove_msg = Float32MultiArray()

# Mirror the camera image
tracker.set_mirror(True)

# Calibrate the controller with the tracker
result = -1
while result != psmove.Tracker_CALIBRATED:
    print 'Trying to calibrate...'
    result = tracker.enable(move)

auto_update_leds = tracker.get_auto_update_leds(move)
print 'Auto-update LEDs is', ('enabled' if auto_update_leds else 'disabled')

# Loop and update the controller
while True:
    # Get the latest input report from the controller
    while move.poll():
        pass

    # Grab the latest image from the camera
    tracker.update_image()
    # Update all tracked controllers
    tracker.update()

    # Check the tracking status
    status = tracker.get_status(move)
    if status == psmove.Tracker_TRACKING:
        x, y, radius = tracker.get_position(move)
        a, b, c, w = move.get_orientation()
        button = 0 if button_pressed else move.get_buttons()
        if move.get_buttons() == 0.0:
            button_pressed = False
        else:
            button_pressed = True

        roll, pitch, yaw = quaternion_to_euler(a, b, c, w)
        # print 'Orientation: (%5.2f, %5.2f, %5.2f, %5.2f)' % (
        #     a, b, c, w)
        print 'Orientation Euler: (%5.2f, %5.2f, %5.2f)' % (
            roll, pitch, yaw)
        print 'buttons: ' + str(button)
        print 'Position: (%5.2f, %5.2f), Radius: %3.2f, Trigger: %3d' % (
            x, y, tracker.distance_from_radius(radius), move.get_trigger())

        psmove_msg.data = [x, y, tracker.distance_from_radius(radius),
                           roll, pitch, yaw, button, move.get_trigger()]
        psmove_topic.publish(psmove_msg)
    elif status == psmove.Tracker_CALIBRATED:
        print 'Not currently tracking.'
    elif status == psmove.Tracker_CALIBRATION_ERROR:
        print 'Calibration error.'
    elif status == psmove.Tracker_NOT_CALIBRATED:
        print 'Controller not calibrated.'
