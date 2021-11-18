#ifndef ESKF_H
#define ESKF_H

#include "../include/utils.h"



class ESKF
{
public:
    ESKF(double acc_n, double gyr_n, double acc_w, double gyr_w, Eigen::Vector3d p_IMU_GNSS);
    StatePtr state_ptr_;
    Eigen::Vector3d p_I_GNSS_;
    Eigen::Vector3d init_lla_;
    bool initialized_;
    std::deque<IMUDataPtr> imu_buffer_;
    IMUDataPtr last_imu_ptr_;

    bool process_IMU_Data(IMUDataPtr imu_data_ptr);
    bool process_GNSS_Data(GNSSDataPtr gnss_data_ptr);
    bool initialize(void);
    void predict(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr);
    void update(GNSSDataPtr gnss_ptr);

private:
    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;
};
using ESKFPtr = std::shared_ptr<ESKF>;

#endif // ESKF_H
