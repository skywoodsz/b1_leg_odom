//
// Created by skywoodsz on 22-10-27.
//

#include <dog_estimator/terrain_estimator.h>

TerrainEstimator::TerrainEstimator(ros::NodeHandle &nh) :
nh_(nh)
{
    norm_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>>(nh_, "/dog/terrain_norm", 100);

    norm_imu_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>>(nh_, "/dog/terrain_imu_norm", 100);

    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dog/terrain_norm_vis", 1);

    marker_real_time_pub_ = nh_.advertise<visualization_msgs::Marker>("/dog/terrain_norm_vis_debug", 1);

    imu_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/dog/terrain_imu_norm_vis_debug", 1);

    terrain_eular_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/dog/terrain_eular", 1);

    terrain_av_eular_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/dog/terrain_av_eular", 1);

    terrain_deatal_z_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/dog/terrain_deatal_z", 1);

    terrain_sum_z_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>("/dog/terrain_sum_z", 1);

    terrain_flag_pub_ = nh.advertise<std_msgs::Bool>("/dog/terrain_flag", 1);

    theta_threshold_ = 3. * M_PI / 180.;

    Reset();

    ros::NodeHandle nh_param = ros::NodeHandle(nh, "param");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<dog_estimator::ParamConfig>>(nh_param);
    dynamic_reconfigure::Server<dog_estimator::ParamConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}

void TerrainEstimator::Reset() {
    for (int leg = 0; leg < 4; ++leg) {
        p_foot_[leg].setZero();
    }
    z_f_.setZero();
    A_pla_.setZero();

    last_postion_.setZero();
    last_publish_ = ros::Time::now();

    // for vis
    id_ = 0;
    marker_array_.markers.clear();

    dealta_z_sum_ = 0.;

    alpha_ = 0.1;
    win_size_ = 1000;

}

void TerrainEstimator::AverageWindows(int win_size, double& av)
{
    if(vt_que_.empty())
    {
        av = 0.0;
    }
    else
    {
        while(vt_que_.size() > win_size)
        {
            vt_que_.pop_front();
        }

        av = 0.0;
        for (auto vt : vt_que_)
        {
            double tmp = vt / vt_que_.size();
            av += tmp;
        }
    }

    
}

void TerrainEstimator::dynamicCallback(dog_estimator::ParamConfig& config, uint32_t /*level*/)
{
    alpha_ = config.alpha;
    theta_threshold_ = config.theta_threshold * M_PI / 180.0;
    win_size_ = config.win_size;

    ROS_INFO("Param update!");
}

void TerrainEstimator::terrainDealtaZ(const RobotState &state, const ros::Duration &period, double& terrain_z)
{

    Eigen::Vector3d gravity_unity(0, 0, 1);
    Eigen::Vector3d terrain_norm_unity = terrain_norm_ / terrain_norm_.norm();
    // Eigen::Vector3d terrain_norm_unity = terrain_imu_norm_ / terrain_imu_norm_.norm();
    double theta = acos(terrain_norm_unity.transpose() * gravity_unity);
    double dealta_z = 0.;
    double av_theta = 0.;
    vt_que_.push_back(theta);
    AverageWindows(win_size_, av_theta);

    Eigen::Vector3d vel = state.linear_vel_;
    bool terrain_flag = false;
    
    if(av_theta > theta_threshold_)
    {
        Eigen::Vector3d terrain_pos = state.pos_;
        Eigen::Matrix3d Rbod = quaternionToRotationMatrix(state.quat_);
        // Eigen::Vector3d delta_pos = Rbod.transpose() * (terrain_pos - terrain_init_pos_);
        Eigen::Vector3d delta_pos = Rbod.transpose() * period.toSec() * vel;
        dealta_z = delta_pos(0) * tan(av_theta);

        // TODO: ??????, ?????? terrain_imu_norm_(0)??????
        // if(terrain_norm_unity(0) > 0)
        //     dealta_z = -dealta_z;
        
        terrain_flag = true;

    }
    else
    {
        terrain_init_pos_ = state.pos_;
        terrain_init_quat_ = state.quat_;
    }



    dealta_z_sum_ += dealta_z;
    terrain_z = dealta_z_sum_;

    geometry_msgs::Vector3Stamped theta_msg, dealta_z_msg, sum_z_msg, av_theta_msg; // 
    ros::Time time = ros::Time::now();
    theta_msg.header.stamp = time;
    dealta_z_msg.header.stamp = time;
    sum_z_msg.header.stamp = time;
    av_theta_msg.header.stamp = time;
    theta_msg.vector.z = theta * 180. / M_PI;
    dealta_z_msg.vector.z = dealta_z;
    sum_z_msg.vector.z = dealta_z_sum_;
    av_theta_msg.vector.z = av_theta * 180. / M_PI; 

    terrain_eular_pub_.publish(theta_msg);
    terrain_deatal_z_pub_.publish(dealta_z_msg);
    terrain_sum_z_pub_.publish(sum_z_msg);
    terrain_av_eular_pub_.publish(av_theta_msg);

    std_msgs::Bool terrain_flag_msg;
    terrain_flag_msg.data = terrain_flag;
    terrain_flag_pub_.publish(terrain_flag_msg);
}

void TerrainEstimator::update(const RobotState &state, double& terrain_z) {
    ros::Time time = ros::Time::now();
    if (time - last_publish_ > ros::Duration(0.01))  // 100Hz
    {
            // methods 1: leg estimation
            for (int leg = 0; leg < 4; ++leg) {
                if(state.contact_state_[leg])
                {
                    // p_foot_[leg] = state.foot_pos_[leg];
                    p_foot_[leg] = state.bfoot_pos_[leg];
                }
                z_f_[leg] = p_foot_[leg][2];
            }

            Eigen::Matrix<double, 4, 3> Wpla;
            Eigen::Vector4d w1 = Eigen::Vector4d::Ones();
            Eigen::Vector4d w2 = Eigen::Vector4d(p_foot_[0][0],  p_foot_[1][0],  p_foot_[2][0],  p_foot_[3][0]);
            Eigen::Vector4d w3 = Eigen::Vector4d(p_foot_[1][1],  p_foot_[1][1],  p_foot_[2][1],  p_foot_[3][1]);

            Wpla.block<4, 1>(0, 0) = w1;
            Wpla.block<4, 1>(0, 1) = w2;
            Wpla.block<4, 1>(0, 2) = w3;

            Eigen::Vector3d A_pla_temp;
            A_pla_temp =Wpla.colPivHouseholderQr().solve(z_f_);

            // double alpha = 0.1; // 0.2
            A_pla_ = alpha_ * A_pla_temp + (1 - alpha_) * A_pla_;

            terrain_norm_ = Eigen::Vector3d(-A_pla_[1], -A_pla_[2], 1);
            terrain_norm_.normalize();

            // methods 2: imu estimation
            Eigen::Quaterniond quat = state.quat_;
            Eigen::Vector3d defalut_norm = Eigen::Vector3d(0, 0, 1);
            terrain_imu_norm_ = quat * defalut_norm;

            // 3. dealta z
            ros::Duration period = time - last_publish_;

            terrainDealtaZ(state, period, terrain_z);

            publish();
            visPublish(state);
            visImuPublish(state);
            if((last_postion_ - state.pos_).norm() > 0.1)
            {
                visArrayPublish(state, id_);
                id_++;
                last_postion_ = state.pos_;
            }
        last_publish_ = time;
    }
    //    std::cout<<"terrain_norm_: "<<"\n"<<terrain_norm_<<std::endl;
    //    std::cout<<"Wpla: "<<"\n"<<Wpla.colPivHouseholderQr().solve(z_f_)<<std::endl;
}

void TerrainEstimator::publish() {
    if(norm_pub_->trylock())
    {
        norm_pub_->msg_.header.stamp = ros::Time::now();
        norm_pub_->msg_.vector.x = terrain_norm_[0];
        norm_pub_->msg_.vector.y = terrain_norm_[1];
        norm_pub_->msg_.vector.z = terrain_norm_[2];
        norm_pub_->unlockAndPublish();
    }

    if(norm_imu_pub_->trylock())
    {
        norm_imu_pub_->msg_.x = terrain_imu_norm_[0];
        norm_imu_pub_->msg_.y = terrain_imu_norm_[1];
        norm_imu_pub_->msg_.z = terrain_imu_norm_[2];
        norm_imu_pub_->unlockAndPublish();
    }
}

void TerrainEstimator::visPublish(const RobotState& state) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "terrain";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0].x = state.pos_.x();
    marker.points[0].y = state.pos_.y();
    marker.points[0].z = 0.;
    marker.points[1].x = state.pos_.x() + terrain_norm_(0);
    marker.points[1].y = state.pos_.y() + terrain_norm_(1);
    marker.points[1].z = terrain_norm_(2);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker_real_time_pub_.publish(marker);
}

void TerrainEstimator::visArrayPublish(const RobotState &state, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "terrain";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0].x = state.pos_.x();
    marker.points[0].y = state.pos_.y();
    marker.points[0].z = 0.;
    marker.points[1].x = state.pos_.x() + terrain_norm_(0) / 10.;
    marker.points[1].y = state.pos_.y() + terrain_norm_(1) / 10.;
    marker.points[1].z = terrain_norm_(2) / 10.;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker_array_.markers.push_back(marker);
    marker_pub_.publish(marker_array_);
}

void TerrainEstimator::visImuPublish(const RobotState &state) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "imu_terrain";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(2);
    marker.points[0].x = state.pos_.x();
    marker.points[0].y = state.pos_.y();
    marker.points[0].z = 0.;
    marker.points[1].x = state.pos_.x() + terrain_imu_norm_(0);
    marker.points[1].y = state.pos_.y() + terrain_imu_norm_(1);
    marker.points[1].z = terrain_imu_norm_(2);
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    imu_marker_pub_.publish(marker);
}











