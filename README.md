# imu_gnss_eskf

The project is to implement an ESKF algorithm to fuse IMU and GNSS data. The theory can be referred to [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508.pdf). The implementation can be referred to [imu_gps_localization](https://github.com/ydsf16/imu_gps_localization). The test dataset can be referred to [EU](https://epan-utbm.github.io/utbm_robocar_dataset/).

### 1. Requirements

The project is tested under UBUNTU 18.04 + ROS melodic.

* nav_msgs is used for ROS publishing. 
* eigen_conversions is used for ROS publishing.
* nmea_navsat_driver is used for GNSS data processing.
* Eigen is used for matrix computation.
* GeographicLib is used for transformation between LLA and ENU.


