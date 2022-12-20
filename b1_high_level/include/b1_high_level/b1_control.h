//
// Created by skywoodsz on 2022/12/18.
//
#ifndef SRC_B1_CONTROL_H
#define SRC_B1_CONTROL_H

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cheetah_msgs/LegContact.h>
#include <cheetah_msgs/LegsState.h>

using namespace UNITREE_LEGGED_SDK;

// high cmd
constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8081;
constexpr char TARGET_IP[] = "192.168.123.220";   // target IP address


//// low cmd
//constexpr uint16_t TARGET_PORT = 8007;
//constexpr uint16_t LOCAL_PORT = 8082;
//constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

struct UnitreeOdomData
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
};

struct UnitreeImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

struct UnitreeFootState
{
    Eigen::Vector3d foot_vel_;
    Eigen::Vector3d bfoot_pos_;
    Eigen::Vector3d bfoot_vel_;
};



class B1Control{
public:
    B1Control(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

private:
    void read(const ros::TimerEvent& event);
    void contactJuistify();
    void publishState();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Timer reader_timer_;
    ros::Timer writer_timer_;

    ros::Time last_publish_;
    tf2_ros::TransformBroadcaster tf_br_;

    std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
    UNITREE_LEGGED_SDK::HighState high_state_{};
    UNITREE_LEGGED_SDK::HighCmd high_cmd_{};

    UnitreeOdomData odom_data_{};
    UnitreeImuData imu_data_{};

    bool contact_state_[4]{};
    int contact_threshold_{};
    double vel_z_contact_threshold_{};

    UnitreeFootState bleg_data_[4]{};

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegContact>> leg_contact_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>> leg_state_pub_;

};




#endif //SRC_B1_CONTROL_H