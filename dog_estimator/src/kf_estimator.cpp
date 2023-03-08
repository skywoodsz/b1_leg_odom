//
// Created by skywoodsz on 2022/12/19.
//

#include <dog_estimator/kf_estimator.h>
#include <dog_estimator/robot_math.h>

KF_ESTIMATOR::KF_ESTIMATOR(const ros::NodeHandle &node_handle,
                           const ros::NodeHandle &private_node_handle) :
nh_(node_handle),
pnh_(private_node_handle)
{
    Reset();

    linear_estimate_ = std::make_shared<LinearKFPosVelEstimator>(nh_);
    terrain_estimator_ = std::make_shared<TerrainEstimator>(nh_);

    state_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>>(nh_, "/test/leg_states", 100);
    
    // 0. get leg state data
    leg_sub_ = nh_.subscribe<cheetah_msgs::LegsState>("/dog/leg_state", 1, &KF_ESTIMATOR::LegStateCallBack, this);

    // 1. get imu data
    initial_yaw_ = 0.;
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/dog/imu_data", 1, &KF_ESTIMATOR::ImuCallBack, this);

    // 2. contact data
    contact_sub_ = nh_.subscribe<cheetah_msgs::LegContact>("/dog/leg_contact", 1, &KF_ESTIMATOR::LegContactCallBack, this);

    timer_ = pnh_.createTimer(ros::Duration(1.0 / 500.0), &KF_ESTIMATOR::update, this);

}

/**
 * state estimate update
 * @param event
 */
void KF_ESTIMATOR::update(const ros::TimerEvent &event) {
    if(imu_flag_ & leg_flag_ & contact_flag_)
    {
        if (linear_estimate_ != nullptr)
            linear_estimate_->update(robot_state_, time_, terrain_z_);

        Body2WorldKine();

        if(terrain_estimator_ != nullptr)
            terrain_estimator_->update(robot_state_, terrain_z_);

        publishState();

    }
}

void KF_ESTIMATOR::publishState()
{
    if(state_pub_->trylock())
    {   
        state_pub_->msg_.foot_pos[0].x = 0.0;

        state_pub_->unlockAndPublish();
    }
}


void KF_ESTIMATOR::Body2WorldKine()
{
    Mat3<double> Rbod = quaternionToRotationMatrix(robot_state_.quat_);

    for (size_t leg = 0; leg < 4; leg++)
    {
        robot_state_.foot_pos_[leg] = robot_state_.pos_ + Rbod * robot_state_.bfoot_pos_[leg];
    } 
}


void KF_ESTIMATOR::Reset() {

    imu_flag_ = false; leg_flag_ = false; contact_flag_ = false;

    robot_state_.pos_.setZero();
    robot_state_.linear_vel_.setZero();

    for (int j = 0; j < 4; ++j) {
        robot_state_.foot_pos_[j].setZero();
        robot_state_.foot_vel_[j].setZero();
        robot_state_.bfoot_pos_[j].setZero();
        robot_state_.bfoot_vel_[j].setZero();
    }

    terrain_z_ = 0.0;
}

/**
 * get leg state data
 * @param msg 
 */
void KF_ESTIMATOR::LegStateCallBack(const cheetah_msgs::LegsState::ConstPtr& msg)
{
    cheetah_msgs::LegsState leg_state = *msg;
    for (size_t leg = 0; leg < 4; leg++)
    {
        robot_state_.bfoot_pos_[leg][0] = leg_state.bfoot_pos[leg].x;
        robot_state_.bfoot_pos_[leg][1] = leg_state.bfoot_pos[leg].y;
        robot_state_.bfoot_pos_[leg][2] = leg_state.bfoot_pos[leg].z;

        robot_state_.bfoot_vel_[leg][0] = leg_state.bfoot_vel[leg].x;
        robot_state_.bfoot_vel_[leg][1] = leg_state.bfoot_vel[leg].y;
        robot_state_.bfoot_vel_[leg][2] = leg_state.bfoot_vel[leg].z;
    }

    time_ = leg_state.header.stamp;

    leg_flag_ = true;
}   

/**
 * get imu data
 * @param msg
 */
void KF_ESTIMATOR::ImuCallBack(const sensor_msgs::Imu_<std::allocator<void>>::ConstPtr &msg) {
    sensor_msgs::Imu imu_ = *msg;

    Eigen::Quaterniond q(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z);
    robot_state_.quat_ = q;

    if (initial_yaw_ == 0)
        initial_yaw_ = quatToRPY(robot_state_.quat_)(2);

    Eigen::Quaternion<double> yaw = RpyToQuat(Vec3<double>(0., 0., -initial_yaw_));

    robot_state_.quat_ *= yaw;
    robot_state_.angular_vel_ << imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z;
    robot_state_.accel_ << imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z;

    imu_flag_ = true;
}

/**
 * get leg contact
 * @param msg
 */
void KF_ESTIMATOR::LegContactCallBack(const cheetah_msgs::LegContact::ConstPtr &msg) {
    cheetah_msgs::LegContact contact_state = *msg;

    for (int j = 0; j < 4; ++j) {
        robot_state_.contact_state_[j] = contact_state.contact_state[j];
    }

    contact_flag_ = true;
}

