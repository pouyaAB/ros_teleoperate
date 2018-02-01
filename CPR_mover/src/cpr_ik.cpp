#include <iostream>
#include "cpr_KinematicMover.h"
#include "cpr_Matrix4.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace std;

cpr_KinematicMover* cpr_instance = new cpr_KinematicMover();

void joint_state_callback(const sensor_msgs::JointState& msg){
    double* joints_state =  new double[4];
    joints_state[0] = msg.position[0];
    joints_state[1] = msg.position[1];
    joints_state[2] = msg.position[2];
    joints_state[3] = msg.position[3];

    cpr_instance->setJointState(joints_state);
    free(joints_state);
}
void cartesian_velocity(const sensor_msgs::JointState& msg){
    double* joints_vel =  new double[6];
    double* joints_diff_vel =  new double[6];
    joints_vel[0] = msg.velocity[0];
    joints_vel[1] = msg.velocity[1];
    joints_vel[2] = msg.velocity[2];
    joints_vel[3] = msg.velocity[3];
    joints_vel[4] = msg.velocity[4];
    joints_vel[5] = msg.velocity[5];

//    cout << joints_vel[0] << ":::"
//         << joints_vel[1] << ":::"
//         << joints_vel[2] << ":::"
//         << joints_vel[3] << ":::" << endl;
    if(joints_vel[0] + joints_vel[1] + joints_vel[2] + joints_vel[3] > 0){
        cpr_instance->SetMotionVec(joints_vel);
        cpr_instance->moveCart();
        cout<<"move cartesian done!"<<endl;

        joints_diff_vel[0] = cpr_instance->currState.j[0] - cpr_instance->setPointState.j[0];
        joints_diff_vel[1] = cpr_instance->currState.j[1] - cpr_instance->setPointState.j[1];
        joints_diff_vel[2] = cpr_instance->currState.j[2] - cpr_instance->setPointState.j[2];
        joints_diff_vel[3] = cpr_instance->currState.j[3] - cpr_instance->setPointState.j[3];

        cout << joints_diff_vel[0] << ":::"
             << joints_diff_vel[1] << ":::"
             << joints_diff_vel[2] << ":::"
             << joints_diff_vel[3] << ":::" << endl;
    }
    free(joints_vel);
    free(joints_diff_vel);
}

int main(int argc, char ** argv){

  cout << "Initializing Ros." << endl;
  // Create ROS node
  ros::init(argc, argv, "leap_al5d");
  ros::NodeHandle n;



  ros::Subscriber joint_states_sub = n.subscribe("/joint_states", 1000, joint_state_callback);
  ros::Subscriber joint_velocity_sub = n.subscribe("/cpr_vel_cartesian", 1000, cartesian_velocity);

  ros::spin();
}