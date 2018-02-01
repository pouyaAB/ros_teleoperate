import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")
    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    print "============ Printing robot state"
    # print robot.get_current_state()
    print "============"
    print group.get_random_pose()
