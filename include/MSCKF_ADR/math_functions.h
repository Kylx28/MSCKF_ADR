#ifndef MSCKF_H
#define MSCKF_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <msckf/math_functions.h>

// This header file includes all of the libraries as well as declaring all of the functions
// and variables used in the program. The actual content of the functions are in make_data.cpp

namespace MSCKF{
class MSCKF {
public:

    MSCKF(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    MSCKF()
    : MSCKF(ros::NodeHandle(), ros::NodeHandle("~")) {}

    void init();
    ~MSCKF(){}

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg);

    Eigen::VectorXd integrate_state(Eigen::VectorXd state);
    // void update();

    // void publish();

    // bool data_lock;
    
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber subIMU_;	
    ros::Publisher pubOdom_;

    sensor_msgs::Imu imu_data;
    nav_msgs::Odometry odom_data;

    // For EKF
    Eigen::VectorXd x_prio;
    Eigen::VectorXd x_post;
    Eigen::MatrixXd F;
    Eigen::VectorXd u;

    Eigen::MatrixXd P_prio;
    Eigen::MatrixXd P_post;
    Eigen::MatrixXd Q;

    // For integration
    Eigen::MatrixXd Omega;

    double dt;

};
}

#endif