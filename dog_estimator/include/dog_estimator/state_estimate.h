//
// Created by skywoodsz on 22-10-25.
//

#ifndef SRC_STATE_ESTIMATE_H
#define SRC_STATE_ESTIMATE_H

#include <dog_estimator/dog_type.h>
#include <dog_estimator/robot_math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <fstream>

using namespace std;

struct LidarPose
{
    ros::Time timeStamp;
    Eigen::Vector3d pos;
};

struct MapTerrainHeight
{
    ros::Time timeStamp;
    double terrain_height;
};


class StateEstimateBase
{
public:
    StateEstimateBase(ros::NodeHandle& nh);
    virtual ~StateEstimateBase(){};
    virtual void update(RobotState& state, ros::Time timeStamp, const double terrain_z);

private:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Path>> path_pub_;

    ros::Time last_publish_;
    Eigen::Vector3d last_position_;
    tf2_ros::TransformBroadcaster tf_br_;
};

class LinearKFPosVelEstimator : public StateEstimateBase
{
public:
    LinearKFPosVelEstimator(ros::NodeHandle& nh);
    void update(RobotState& state, ros::Time timeStamp, const double terrain_z);

private:
    void LidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void MapTerrainHeigtCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

    ros::Subscriber lidar_sub_, map_sub_; 
    LidarPose lidar_pose_;
    MapTerrainHeight map_height_;

    bool lidar_updated_flag_, lidar_init_flag_;
    bool map_updated_flag_, map_init_flag_;

    Eigen::Matrix<double, 18, 1> x_hat_;
    Eigen::Matrix<double, 12, 1> ps_;
    Eigen::Matrix<double, 12, 1> vs_;
    Eigen::Matrix<double, 18, 18> a_;
    Eigen::Matrix<double, 18, 18> q_;
    Eigen::Matrix<double, 18, 18> p_;
    Eigen::Matrix<double, 34, 34> r_; // 28, 28
    Eigen::Matrix<double, 18, 3> b_;
    Eigen::Matrix<double, 34, 18> c_; // 28, 18
};





#endif //SRC_STATE_ESTIMATE_H
