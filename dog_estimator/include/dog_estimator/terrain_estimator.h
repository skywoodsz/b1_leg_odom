//
// Created by skywoodsz on 22-10-27.
//

#ifndef SRC_TERRAIN_ESTIMATOR_H
#define SRC_TERRAIN_ESTIMATOR_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <realtime_tools/realtime_publisher.h>

#include <dog_estimator/dog_type.h>
#include <dog_estimator/robot_math.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <dynamic_reconfigure/server.h>
#include "dog_estimator/ParamConfig.h"

#include <deque>


class TerrainEstimator
{
public:
    TerrainEstimator(ros::NodeHandle& nh);
    void update(const RobotState& state, double& terrain_z);
    void Reset();

private:
    void publish();
    void visPublish(const RobotState& state);
    void visImuPublish(const RobotState& state);
    void visArrayPublish(const RobotState& state, int id);
    void terrainDealtaZ(const RobotState &state, const ros::Duration &period, double& terrain_z);
    void dynamicCallback(dog_estimator::ParamConfig& config, uint32_t /*level*/);
    void AverageWindows(int win_size, double& av);

    ros::NodeHandle nh_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>> norm_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> norm_imu_pub_;
    ros::Publisher marker_pub_, imu_marker_pub_;
    ros::Publisher marker_real_time_pub_;
    ros::Publisher terrain_eular_pub_;
    ros::Publisher terrain_deatal_z_pub_;
    ros::Publisher terrain_sum_z_pub_;
    ros::Publisher terrain_flag_pub_;
    ros::Publisher terrain_av_eular_pub_;

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

    double dealta_z_sum_;

    std::shared_ptr<dynamic_reconfigure::Server<dog_estimator::ParamConfig>> dynamic_srv_{};
    double alpha_;
    int win_size_;

    std::deque<double> vt_que_;
};


#endif //SRC_TERRAIN_ESTIMATOR_H
