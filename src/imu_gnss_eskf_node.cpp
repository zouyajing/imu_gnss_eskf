#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>



#include "../include/utils.h"
#include "../include/eskf.h"

class ESKF_Fusion
{
public:
    ESKF_Fusion(ros::NodeHandle nh){
        double acc_n, gyr_n, acc_w, gyr_w;
        nh.param("acc_noise", acc_n, 1e-2);
        nh.param("gyr_noise", gyr_n, 1e-4);
        nh.param("acc_bias_noise", acc_w, 1e-6);
        nh.param("gyr_bias_noise", gyr_w, 1e-8);
        double x, y, z;
        nh.param("p_I_GNSS_x", x, 0.);
        nh.param("p_I_GNSS_y", y, 0.);
        nh.param("p_I_GNSS_z", z, 0.);
        const Eigen::Vector3d p_I_GNSS(x, y, z);
        eskf_ptr_ = std::make_shared<ESKF>(acc_n, gyr_n, acc_w, gyr_w, p_I_GNSS);

        // ROS sub & pub
        std::string topic_imu = "/imu/data";
        std::string topic_gps = "/fix";

        imu_sub_ = nh.subscribe(topic_imu, 10, &ESKF_Fusion::imu_callback, this);
        gnss_sub_ = nh.subscribe(topic_gps, 10, &ESKF_Fusion::gnss_callback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);

        // log files
        file_gnss_.open("/home/rick/gnss.csv");
        file_state_.open("/home/rick/fused_state.csv");

        std::cout<<"[ESKF] Node starts."<<std::endl;
    }
    ~ESKF_Fusion()
    {
        file_gnss_.close();
        file_state_.close();
    }
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gnss_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
    void publish_save_state();

private:
   ros::Subscriber imu_sub_;
   ros::Subscriber gnss_sub_;
   ros::Publisher path_pub_;
   ros::Publisher odom_pub_;
   nav_msgs::Path nav_path_;

   ESKFPtr eskf_ptr_;

   // log files
   std::ofstream file_gnss_;
   std::ofstream file_state_;
};

void ESKF_Fusion::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    IMUDataPtr imu_data_ptr = std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;
    bool ok = eskf_ptr_->process_IMU_Data(imu_data_ptr);
    if(!ok) return;


    file_state_<< std::fixed << std::setprecision(15)
               << imu_msg->header.stamp.toSec() << std::endl;

}

void ESKF_Fusion::gnss_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg)
{
    if(gnss_msg->status.status != 2)
    {
        std::cout<<"[ESKF] Bad GNSS data."<<std::endl;
        return;
    }
    GNSSDataPtr gnss_data_ptr = std::make_shared<GNSSData>();
    gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gnss_msg->latitude;
    gnss_data_ptr->lla[1] = gnss_msg->longitude;
    gnss_data_ptr->lla[2] = gnss_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());
    bool ok = eskf_ptr_->process_GNSS_Data(gnss_data_ptr);
    if(!ok) return;
    file_gnss_ << std::fixed << std::setprecision(15)
              << gnss_msg->header.stamp.toSec() << std::endl;

}

void ESKF_Fusion::publish_save_state()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gnss_eskf_node");
    ros::NodeHandle nh;
    ESKF_Fusion node(nh);
    ros::spin();
}
