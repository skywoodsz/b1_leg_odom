//
// Created by skywoodsz on 22-10-27.
//

#ifndef SRC_TERRAIN_ESTIMATOR_H
#define SRC_TERRAIN_ESTIMATOR_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <realtime_tools/realtime_publisher.h>

#include <dog_estimator/dog_type.h>
#include <dog_estimator/robot_math.h>

#include <geometry_msgs/Vector3Stamped.h>

class TerrainEstimator
{
public:
    TerrainEstimator(ros::NodeHandle& nh);
    void update(const RobotState& state);
    void Reset();

private:
    void publish();
    void visPublish(const RobotState& state);
    void visImuPublish(const RobotState& state);
    void visArrayPublish(const RobotState& state, int id);
    void terrainDealtaZ(const RobotState &state);

    ros::NodeHandle nh_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> norm_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> norm_imu_pub_;
    ros::Publisher marker_pub_, imu_marker_pub_;
    ros::Publisher marker_real_time_pub_;
    ros::Publisher terrain_eular_pub_;
    ros::Publisher terrain_deatal_z_pub_;



    Eigen::Vector3d p_foot_[4];
    Eigen::Vector3d A_pla_;
    Eigen::Vector4d z_f_;
    Eigen::Vector3d terrain_norm_;
    Eigen::Vector3d terrain_imu_norm_;

    int id_;
    visualization_msgs::MarkerArray marker_array_;

    ros::Time last_publish_;
    Eigen::Vector3d last_postion_;

    double theta_threshold_;
    Eigen::Vector3d terrain_init_pos_;
    Eigen::Quaterniond terrain_init_quat_;
};


#endif //SRC_TERRAIN_ESTIMATOR_H
