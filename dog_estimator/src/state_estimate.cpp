//
// Created by skywoodsz on 22-10-25.
//

#include "dog_estimator/state_estimate.h"

StateEstimateBase::StateEstimateBase(ros::NodeHandle& nh)
{
    odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(nh, "/dog/odom", 100);
    path_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Path>>(nh, "/dog/legged_path", 100);
}

void StateEstimateBase::update(RobotState& state, ros::Time timeStamp, const double terrain_z)
{
    ros::Time time = timeStamp;
    if (odom_pub_->trylock())
    {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.orientation.x = state.quat_.x();
        odom_pub_->msg_.pose.pose.orientation.y = state.quat_.y();
        odom_pub_->msg_.pose.pose.orientation.z = state.quat_.z();
        odom_pub_->msg_.pose.pose.orientation.w = state.quat_.w();
        odom_pub_->msg_.pose.pose.position.x = state.pos_[0];
        odom_pub_->msg_.pose.pose.position.y = state.pos_[1];
        odom_pub_->msg_.pose.pose.position.z = state.pos_[2];
        odom_pub_->msg_.twist.twist.angular.x = state.angular_vel_[0];
        odom_pub_->msg_.twist.twist.angular.y = state.angular_vel_[1];
        odom_pub_->msg_.twist.twist.angular.z = state.angular_vel_[2];
        odom_pub_->msg_.twist.twist.linear.x = state.linear_vel_[0];
        odom_pub_->msg_.twist.twist.linear.y = state.linear_vel_[1];
        odom_pub_->msg_.twist.twist.linear.z = state.linear_vel_[2];
        odom_pub_->unlockAndPublish();
    }

    // tf
    // geometry_msgs::TransformStamped transform_stamped;
    // transform_stamped.header.stamp = time;
    // transform_stamped.header.frame_id = "odom";
    // transform_stamped.child_frame_id = "base_link";
    // transform_stamped.transform.translation.x = state.pos_[0];
    // transform_stamped.transform.translation.y = state.pos_[1];
    // transform_stamped.transform.translation.z = state.pos_[2];
    // transform_stamped.transform.rotation.x = state.quat_.x();
    // transform_stamped.transform.rotation.y = state.quat_.y();
    // transform_stamped.transform.rotation.z = state.quat_.z();
    // transform_stamped.transform.rotation.w = state.quat_.w();
    // tf_br_.sendTransform(transform_stamped);

    // path
    Eigen::Vector3d current_position = Eigen::Vector3d(state.pos_[0],
                                                        state.pos_[1],
                                                        state.pos_[2]);
    if((last_position_ - current_position).norm() > 0.1)
    {
        last_position_ = current_position;
        if (path_pub_->trylock()) {
            geometry_msgs::PoseStamped legged_pose;
            legged_pose.header.stamp = time;
            legged_pose.header.frame_id = "odom";
            legged_pose.pose.position.x = state.pos_[0];
            legged_pose.pose.position.y = state.pos_[1];
            legged_pose.pose.position.z = state.pos_[2];

            legged_pose.pose.orientation.x = state.quat_.x();
            legged_pose.pose.orientation.y = state.quat_.y();
            legged_pose.pose.orientation.z = state.quat_.z();
            legged_pose.pose.orientation.w = state.quat_.w();

            path_pub_->msg_.header.stamp = ros::Time::now();
            path_pub_->msg_.header.frame_id = "odom";
            path_pub_->msg_.poses.push_back(legged_pose);

            path_pub_->unlockAndPublish();
        }
    }
    
}

LinearKFPosVelEstimator::LinearKFPosVelEstimator(ros::NodeHandle& nh) : StateEstimateBase(nh)
{
    double dt = 0.002; // shoud be feedback dt
    x_hat_.setZero(); // w_pcom, w_vcom, w_pfoot
    ps_.setZero();
    vs_.setZero();
    a_.setZero();
    a_.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    a_.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    a_.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
    b_.setZero();
    b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<double, 3, 3>::Identity();
    b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
    c1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
    c2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    c_.setZero();
    c_.block(0, 0, 3, 6) = c1;
    c_.block(3, 0, 3, 6) = c1;
    c_.block(6, 0, 3, 6) = c1;
    c_.block(9, 0, 3, 6) = c1;
    c_.block(0, 6, 12, 12) = - Eigen::Matrix<double, 12, 12>::Identity();
    c_.block(12, 0, 3, 6) = c2;
    c_.block(15, 0, 3, 6) = c2;
    c_.block(18, 0, 3, 6) = c2;
    c_.block(21, 0, 3, 6) = c2;
    c_(27, 17) = 1.0;
    c_(26, 14) = 1.0;
    c_(25, 11) = 1.0;
    c_(24, 8) = 1.0;
    // lidar
    c_(28, 0) = 1.0;
    c_(29, 1) = 1.0;
    // map height
    c_(30, 8) = 1.0;
    c_(31, 11) = 1.0;
    c_(32, 14) = 1.0;
    c_(33, 17) = 1.0;

    p_.setIdentity();
    p_ = 100. * p_;
    q_.setIdentity();
    q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<double, 12, 12>::Identity();
    r_.setIdentity();

    lidar_pose_ = {};
    map_height_ = {};
    lidar_updated_flag_ = false;
    lidar_init_flag_ = false;
    map_updated_flag_ = false;
    map_init_flag_ = false;

    lidar_sub_ = nh.subscribe("/lio_sam/mapping/odometry", 1, &LinearKFPosVelEstimator::LidarOdomCallback, this);
    map_sub_ = nh.subscribe("/lio_sam/map/z", 1, &LinearKFPosVelEstimator::MapTerrainHeigtCallback, this);
}

void LinearKFPosVelEstimator::update(RobotState& state, ros::Time timeStamp, const double terrain_z)
{
    // predict
    double imu_process_noise_position = 0.02;
    double imu_process_noise_velocity = 0.02;
    double foot_process_noise_position = 0.002;
    // observer
    double foot_sensor_noise_position = 0.005; // 0.001
    double foot_sensor_noise_velocity = 0.1;
    double foot_height_sensor_noise = 0.010; // 0.001; 0.01;
    double lidar_sensor_noise = 0.001;
    double map_terrain_height_noise = 0.001;

    double high_suspect_number = 100;


    // lidar
    Eigen::Vector3d lidar_pos;
    if(lidar_init_flag_)
    {
        if(lidar_updated_flag_){
                lidar_pos = lidar_pose_.pos;
                lidar_updated_flag_= false;
        }
        else{
            lidar_pos = lidar_pose_.pos + state.linear_vel_ * (timeStamp - lidar_pose_.timeStamp).toSec();

            lidar_sensor_noise *= high_suspect_number;
            lidar_sensor_noise *= high_suspect_number;
        }
    }
    else{
        lidar_pos = lidar_pose_.pos;
        lidar_sensor_noise *= high_suspect_number;
        lidar_sensor_noise *= high_suspect_number;
    }

    // map
    double terrain_height;
    if(map_updated_flag_)
    {
        terrain_height = map_height_.terrain_height;
        map_updated_flag_ = false;
        std::cout<<"terrain_height: "<<terrain_height<<std::endl;
    }
    else{
        map_terrain_height_noise *= high_suspect_number;
    }

    Mat3<double> Rbod = quaternionToRotationMatrix(state.quat_);

    Eigen::Matrix<double, 18, 18> q = Eigen::Matrix<double, 18, 18>::Identity();
    q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position;
    q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity;
    q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * foot_process_noise_position;

    Eigen::Matrix<double, 34, 34> r = Eigen::Matrix<double, 34, 34>::Identity(); // 28 28
    r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * foot_sensor_noise_position;
    r.block(12, 12, 12, 12) = r_.block(12, 12, 12, 12) * foot_sensor_noise_velocity;
    r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * foot_height_sensor_noise;
    // lidar
    r.block(28, 28, 2, 2) = r_.block(28, 28, 2, 2) * lidar_sensor_noise;
    // map height
    r.block(30, 30, 4, 4) = r_.block(30, 30, 4, 4) * map_terrain_height_noise;

    Vec4<double> pzs = Vec4<double>::Zero();
    Vec4<double> terrain_height_vec = Vec4<double>::Zero();

    for (int i = 0; i < 4; i++)
    {
        int i1 = 3 * i;

        int q_index = 6 + i1;
        int r_index1 = i1;
        int r_index2 = 12 + i1;
        int r_index3 = 24 + i;

        
        q.block(q_index, q_index, 3, 3) =
                (state.contact_state_[i] ? 1. : high_suspect_number) * q.block(q_index, q_index, 3, 3);
        r.block(r_index1, r_index1, 3, 3) = 1. * r.block(r_index1, r_index1, 3, 3);
        r.block(r_index2, r_index2, 3, 3) =
                (state.contact_state_[i] ? 1. : high_suspect_number) * r.block(r_index2, r_index2, 3, 3);
        r(r_index3, r_index3) = (state.contact_state_[i] ? 1. : high_suspect_number) * r(r_index3, r_index3);


        Vec3<double> p_rel = state.bfoot_pos_[i]; // b_pfoot
        Vec3<double> dp_rel = state.bfoot_vel_[i]; // b_vfoot
        Vec3<double> p_f = Rbod * p_rel;
        Vec3<double> omegaBody = state.angular_vel_;
        Vec3<double> dp_f = Rbod * (omegaBody.cross(p_rel) + dp_rel);

        ps_.segment(i1, 3) = -p_f;
        vs_.segment(i1, 3) = -dp_f;


        pzs(i) = terrain_z;
        terrain_height_vec(i) = terrain_height;
        // pzs(i) = 0.;

    }

    Vec3<double> g(0, 0, -9.81);
    Vec3<double> accel = Rbod * state.accel_ + g;

    Eigen::Matrix<double, 34, 1> y; // Eigen::Matrix<double, 28, 1> y;
    y << ps_, vs_, pzs, lidar_pos[0], lidar_pos[1], terrain_height_vec; 

    x_hat_ = a_ * x_hat_ + b_ * accel;
    Eigen::Matrix<double, 18, 18> at = a_.transpose();
    Eigen::Matrix<double, 18, 18> pm = a_ * p_ * at + q;
    Eigen::Matrix<double, 18, 34> ct = c_.transpose();
    Eigen::Matrix<double, 34, 1> y_model = c_ * x_hat_;
    Eigen::Matrix<double, 34, 1> ey = y - y_model;
    Eigen::Matrix<double, 34, 34> s = c_ * pm * ct + r;

    Eigen::Matrix<double, 34, 1> s_ey = s.lu().solve(ey);
    x_hat_ += pm * ct * s_ey;

    Eigen::Matrix<double, 34, 18> s_c = s.lu().solve(c_);
    p_ = (Eigen::Matrix<double, 18, 18>::Identity() - pm * ct * s_c) * pm;

    Eigen::Matrix<double, 18, 18> pt = p_.transpose();
    p_ = (p_ + pt) / 2.0;

    if (p_.block(0, 0, 2, 2).determinant() > 0.000001)
    {
        p_.block(0, 2, 2, 16).setZero();
        p_.block(2, 0, 16, 2).setZero();
        p_.block(0, 0, 2, 2) /= 10.;
    }

    state.pos_ = x_hat_.block(0, 0, 3, 1);
    state.linear_vel_ = x_hat_.block(3, 0, 3, 1);

    StateEstimateBase::update(state, timeStamp, terrain_z);
}

void LinearKFPosVelEstimator::MapTerrainHeigtCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    geometry_msgs::Vector3Stamped map_height_msg = *msg;

    map_height_.timeStamp = map_height_msg.header.stamp;
    map_height_.terrain_height = map_height_msg.vector.z;

    map_updated_flag_ = true;
    map_init_flag_ = true;
}

void LinearKFPosVelEstimator::LidarOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry odom_msg = *msg;
    ros::Time timeStamp = odom_msg.header.stamp;

    lidar_pose_.timeStamp = timeStamp;
    lidar_pose_.pos[0] = odom_msg.pose.pose.position.x;
    lidar_pose_.pos[1] = odom_msg.pose.pose.position.y;
    lidar_pose_.pos[2] = odom_msg.pose.pose.position.z;

    lidar_init_flag_ = true;
    lidar_updated_flag_ = true;
}


