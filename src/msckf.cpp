#include "ros/ros.h"
#include <cmath>
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <random>

#include "MSCKF_ADR/msckf.h"

namespace MSCKF {

MSCKF::MSCKF(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {

    subIMU_ = nh_.subscribe<sensor_msgs::Imu>("imu_topic", 5, &MSCKF::imuCallback, this);
	  ROS_INFO("MSCKF INIT : subscribe to IMU");

    pubOdom_ = nh_.advertise<nav_msgs::Odometry>("msckf/odom", 1000);
    ROS_INFO("MSCKF INIT : publish to Odom");

}

void MSCKF::init(){

    x_prio = Eigen::VectorXd::Zero(16);
    //  The 16 states are
    //  0 q_x  rotation quaternion x
    //  1 q_y  rotation quaternion y
    //  2 q_z  rotation quaternion z
    //  3 q_w  rotation quaternion w
    //  4 bg_x gyroscope bias x
    //  5 bg_y gyroscope bias y
    //  6 bg_z gyroscope bias z
    //  7 v_x  velocity x
    //  8 v_y  velocity y
    //  9 v_z  velocity z
    // 10 ba_x accelerometer bias x
    // 11 ba_y accelerometer bias y
    // 12 ba_z accelerometer bias z
    // 13 p_x  position x
    // 14 p_y  position y
    // 15 p_z  position z

    x_post = Eigen::VectorXd::Zero(16);

    P_prio.resize(9,9);
    float p = 0.5;
    P_prio << p, 0, 0, 0, 0, 0, 0, 0, 0,
              0, p, 0, 0, 0, 0, 0, 0, 0,
              0, 0, p, 0, 0, 0, 0, 0, 0,
              0, 0, 0, p, 0, 0, 0, 0, 0,
              0, 0, 0, 0, p, 0, 0, 0, 0,
              0, 0, 0, 0, 0, p, 0, 0, 0,
              0, 0, 0, 0, 0, 0, p, 0, 0,
              0, 0, 0, 0, 0, 0, 0, p, 0,
              0, 0, 0, 0, 0, 0, 0, 0, p;

    P_post.resize(9,9);
    P_post = P_prio;

    Q.resize(9,9);
    float q = 0.1;
    Q << q, 0, 0, 0, 0, 0, 0, 0, 0,
         0, q, 0, 0, 0, 0, 0, 0, 0,
         0, 0, q, 0, 0, 0, 0, 0, 0,
         0, 0, 0, q, 0, 0, 0, 0, 0,
         0, 0, 0, 0, q, 0, 0, 0, 0,
         0, 0, 0, 0, 0, q, 0, 0, 0,
         0, 0, 0, 0, 0, 0, q, 0, 0,
         0, 0, 0, 0, 0, 0, 0, q, 0,
         0, 0, 0, 0, 0, 0, 0, 0, q;
    u = Eigen::VectorXd::Zero(6);

 	ROS_INFO("Initialized!");
}

Eigen::Matrix3d MSCKF::skew_symm(Eigen::Vector3d vector){
  Eigen::Matrix3d skew_matrix;
  skew_matrix << 0, -vector(2), vector(1),
                 vector(2), 0, -vector(0),
                -vector(1), vector(0), 0;
  return skew_matrix;
}

void MSCKF::integrate_state(){

  Omega.resize(4,4);
  Omega <<   0, -u(5),  u(4), u(3),
          u(5),     0, -u(3), u(4),
         -u(4),  u(3),     0, u(5),
         -u(3), -u(4), -u(5),    0;

}

void MSCKF::imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg){

}
}