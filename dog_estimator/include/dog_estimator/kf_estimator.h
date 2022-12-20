//
// Created by skywoodsz on 2022/12/19.
//

#ifndef SRC_KF_ESTIMATOR_H
#define SRC_KF_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <cheetah_msgs/MotorState.h>
#include <cheetah_msgs/LegContact.h>
#include <cheetah_msgs/LegsState.h>

#include <dog_estimator/dog_type.h>
#include <dog_estimator/state_estimate.h>
#include <dog_estimator/terrain_estimator.h>

class KF_ESTIMATOR{
public:
    KF_ESTIMATOR(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);
    void Reset();

    RobotState robot_state_;

private:
    void ImuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void LegContactCallBack(const cheetah_msgs::LegContact::ConstPtr& msg);
    void LegStateCallBack(const cheetah_msgs::LegsState::ConstPtr& msg);
    void update(const ros::TimerEvent& event);
    void publishState();
    void Body2WorldKine();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Time last_publish_;
    ros::Timer timer_;
    ros::Time time_;

    ros::Subscriber imu_sub_;
    ros::Subscriber leg_sub_;
    ros::Subscriber contact_sub_;

    bool imu_flag_, leg_flag_, contact_flag_;

    realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_buffer_;

    double initial_yaw_;

    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>> state_pub_;

    std::shared_ptr<StateEstimateBase> linear_estimate_;
    std::shared_ptr<TerrainEstimator> terrain_estimator_;

};

#endif //SRC_KF_ESTIMATOR_H