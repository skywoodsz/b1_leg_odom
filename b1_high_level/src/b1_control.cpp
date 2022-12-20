//
// Created by skywoodsz on 2022/12/18.
//

#include "b1_high_level/b1_control.h"
#include "b1_high_level/robot_math.h"


B1Control::B1Control(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle) :
nh_(node_handle),
pnh_(private_node_handle)
{
    InitEnvironment();
    
    // init udp
    udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(LOCAL_PORT, TARGET_IP,TARGET_PORT,
                                                     sizeof(UNITREE_LEGGED_SDK::HighCmd),
                                                     sizeof(UNITREE_LEGGED_SDK::HighState));
    high_cmd_ = {0};
    udp_->InitCmdData(high_cmd_);
    udp_->Send();

    ROS_INFO("\033[1;32m----> UDP Init!\033[0m");

    contact_threshold_ = 10;
    vel_z_contact_threshold_ = 0.01;

    odom_pub_ =
            std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(nh_, "/dog/unitree_odom", 100);
    
    imu_pub_ = 
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::Imu>>(nh_, "/dog/imu_data", 100);

    leg_contact_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegContact>>(nh_, "/dog/leg_contact", 100);

    leg_state_pub_ = 
        std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>>(nh_, "/dog/leg_state", 100);

    reader_timer_ = pnh_.createTimer(ros::Duration(1.0 / 500.0), &B1Control::read, this);
}

/**
 * 
 * TODO: read MotorState and publish
 * read high state of b1 and publish the odom
 * @param event
 */
void B1Control::read(const ros::TimerEvent &event)
{
    udp_->Recv();
    udp_->GetRecv(high_state_);

    // IMU
    imu_data_.ori[1] = high_state_.imu.quaternion[1];
    imu_data_.ori[2] = high_state_.imu.quaternion[2];
    imu_data_.ori[3] = high_state_.imu.quaternion[3];
    imu_data_.ori[0] = high_state_.imu.quaternion[0];
    
    imu_data_.angular_vel[0] = high_state_.imu.gyroscope[0];
    imu_data_.angular_vel[1] = high_state_.imu.gyroscope[1];
    imu_data_.angular_vel[2] = high_state_.imu.gyroscope[2];
    imu_data_.linear_acc[0] = high_state_.imu.accelerometer[0];
    imu_data_.linear_acc[1] = high_state_.imu.accelerometer[1];
    imu_data_.linear_acc[2] = high_state_.imu.accelerometer[2];

    // odom
    odom_data_.pos[0] = high_state_.position[0];
    odom_data_.pos[1] = high_state_.position[1];
    odom_data_.pos[2] = high_state_.position[2];

    odom_data_.linear_vel[0] = high_state_.velocity[0];
    odom_data_.linear_vel[1] = high_state_.velocity[1];
    odom_data_.linear_vel[2] = high_state_.velocity[2];

    odom_data_.ori.w() = high_state_.imu.quaternion[3];
    odom_data_.ori.x() = high_state_.imu.quaternion[0];
    odom_data_.ori.y() = high_state_.imu.quaternion[1];
    odom_data_.ori.z() = high_state_.imu.quaternion[2];

    odom_data_.angular_vel[0] = high_state_.imu.gyroscope[0];
    odom_data_.angular_vel[1] = high_state_.imu.gyroscope[1];
    odom_data_.angular_vel[2] = high_state_.yawSpeed;

    // leg state
    // FL
    bleg_data_[0].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].x;
    bleg_data_[0].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].y;
    bleg_data_[0].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].z;

    bleg_data_[0].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].x;
    bleg_data_[0].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].y;
    bleg_data_[0].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].z;
    // FR
    bleg_data_[1].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].x;
    bleg_data_[1].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].y;
    bleg_data_[1].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].z;

    bleg_data_[1].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].x;
    bleg_data_[1].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].y;
    bleg_data_[1].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].z;
    // RL
    bleg_data_[2].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].x;
    bleg_data_[2].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].y;
    bleg_data_[2].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].z;

    bleg_data_[2].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].x;
    bleg_data_[2].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].y;
    bleg_data_[2].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].z;
    // RR
    bleg_data_[3].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].x;
    bleg_data_[3].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].y;
    bleg_data_[3].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].z;

    bleg_data_[3].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].x;
    bleg_data_[3].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].y;
    bleg_data_[3].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].z;

    publishState();

    // leg contact
    contactJuistify();
    // contact_state_[0] = high_state_.footForce[UNITREE_LEGGED_SDK::FL_] > contact_threshold_;
    // contact_state_[1] = high_state_.footForce[UNITREE_LEGGED_SDK::FR_] > contact_threshold_;
    // contact_state_[2] = high_state_.footForce[UNITREE_LEGGED_SDK::RL_] > contact_threshold_;
    // contact_state_[3] = high_state_.footForce[UNITREE_LEGGED_SDK::RR_] > contact_threshold_;


    // debug
    // std::cout<<"footForce[UNITREE_LEGGED_SDK::FL_]: "<<high_state_.footForce[UNITREE_LEGGED_SDK::FL_]<<std::endl;
}


void B1Control::contactJuistify()
{
    Eigen::Vector3d v_com = Eigen::Vector3d(high_state_.velocity[0], high_state_.velocity[1], high_state_.velocity[2]);
    Eigen::Vector3d w_com = Eigen::Vector3d(high_state_.imu.gyroscope[0], high_state_.imu.gyroscope[1], high_state_.imu.gyroscope[2]);
    Eigen::Matrix3d Rbod = quaternionToRotationMatrix(Eigen::Quaterniond(odom_data_.ori.w(), odom_data_.ori.x(), odom_data_.ori.y(), odom_data_.ori.z()));

   
    
    for (size_t leg = 0; leg < 4; leg++)
    {
       Eigen::Vector3d v_foot = Eigen::Vector3d(bleg_data_[leg].bfoot_vel_[0],
                                                bleg_data_[leg].bfoot_vel_[1],
                                                bleg_data_[leg].bfoot_vel_[2]);

       Eigen::Vector3d p_foot = Eigen::Vector3d(bleg_data_[leg].bfoot_pos_[0],
                                                bleg_data_[leg].bfoot_pos_[1],
                                                bleg_data_[leg].bfoot_pos_[2]);

       Eigen::Vector3d v_foot_in_world = v_com + Rbod * (w_com.cross(p_foot) + v_foot);

       bleg_data_[leg].foot_vel_ = v_foot_in_world;
    }
    


    Eigen::Vector4d leg_vel_z = Eigen::Vector4d(bleg_data_[0].foot_vel_[2],
                                                bleg_data_[1].foot_vel_[2],
                                                bleg_data_[2].foot_vel_[2],
                                                bleg_data_[3].foot_vel_[2]
                                                );

    for (size_t leg = 0; leg < 4; leg++)
    {
        contact_state_[leg] = abs(leg_vel_z[leg]) < vel_z_contact_threshold_;
    }

}

/**
 * publish the odom, imu, and leg contact 
 */
void B1Control::publishState() {

    if (imu_pub_->trylock()) {
        imu_pub_->msg_.header.stamp = ros::Time::now();
        imu_pub_->msg_.header.frame_id = "imu_link";

        imu_pub_->msg_.orientation.x = imu_data_.ori[0];
        imu_pub_->msg_.orientation.y = imu_data_.ori[1];
        imu_pub_->msg_.orientation.z = imu_data_.ori[2];
        imu_pub_->msg_.orientation.w = imu_data_.ori[3];

        imu_pub_->msg_.linear_acceleration.x = imu_data_.linear_acc[0];
        imu_pub_->msg_.linear_acceleration.y = imu_data_.linear_acc[1];
        imu_pub_->msg_.linear_acceleration.z = imu_data_.linear_acc[2];

        imu_pub_->msg_.angular_velocity.x = imu_data_.angular_vel[0];
        imu_pub_->msg_.angular_velocity.y = imu_data_.angular_vel[1];
        imu_pub_->msg_.angular_velocity.z = imu_data_.angular_vel[2];

        imu_pub_->unlockAndPublish();
    }

    if(leg_contact_pub_->trylock())
    {
        for (int i = 0; i < 4; ++i) {
            leg_contact_pub_->msg_.contact_state[i] = contact_state_[i];
        }
        leg_contact_pub_->unlockAndPublish();
    }

    if (odom_pub_->trylock()) {
        odom_pub_->msg_.header.stamp = ros::Time::now();
        odom_pub_->msg_.pose.pose.orientation.x = odom_data_.ori.x();
        odom_pub_->msg_.pose.pose.orientation.y = odom_data_.ori.y();
        odom_pub_->msg_.pose.pose.orientation.z = odom_data_.ori.z();
        odom_pub_->msg_.pose.pose.orientation.w = odom_data_.ori.w();

        odom_pub_->msg_.pose.pose.position.x = odom_data_.pos[0];
        odom_pub_->msg_.pose.pose.position.y = odom_data_.pos[1];
        odom_pub_->msg_.pose.pose.position.z = odom_data_.pos[2];

        odom_pub_->msg_.twist.twist.angular.x = odom_data_.linear_vel[0];
        odom_pub_->msg_.twist.twist.angular.y = odom_data_.linear_vel[1];
        odom_pub_->msg_.twist.twist.angular.z = odom_data_.linear_vel[2];

        odom_pub_->msg_.twist.twist.linear.x = odom_data_.angular_vel[0];
        odom_pub_->msg_.twist.twist.linear.y = odom_data_.angular_vel[1];
        odom_pub_->msg_.twist.twist.linear.z = odom_data_.angular_vel[2];

        odom_pub_->unlockAndPublish();
    }

    if(leg_state_pub_->trylock())
    {
        leg_state_pub_->msg_.header.stamp = ros::Time::now();
        for (size_t leg = 0; leg < 4; leg++)
        {
            leg_state_pub_->msg_.bfoot_pos[leg].x = bleg_data_[leg].bfoot_pos_[0];
            leg_state_pub_->msg_.bfoot_pos[leg].y = bleg_data_[leg].bfoot_pos_[1];
            leg_state_pub_->msg_.bfoot_pos[leg].z = bleg_data_[leg].bfoot_pos_[2];

            leg_state_pub_->msg_.bfoot_vel[leg].x = bleg_data_[leg].bfoot_vel_[0];
            leg_state_pub_->msg_.bfoot_vel[leg].y = bleg_data_[leg].bfoot_vel_[1];
            leg_state_pub_->msg_.bfoot_vel[leg].z = bleg_data_[leg].bfoot_vel_[2];

            leg_state_pub_->msg_.foot_vel[leg].x = bleg_data_[leg].foot_vel_[0];
            leg_state_pub_->msg_.foot_vel[leg].y = bleg_data_[leg].foot_vel_[1];
            leg_state_pub_->msg_.foot_vel[leg].z = bleg_data_[leg].foot_vel_[2];
        }
        leg_state_pub_->unlockAndPublish();
    }
}