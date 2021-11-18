
#include "../include/eskf.h"
ESKF::ESKF(double acc_n, double gyro_n, double acc_w, double gyro_w, Eigen::Vector3d p_IMU_GNSS)
{
    acc_noise_ = acc_n;
    gyro_noise_ = gyro_n;
    acc_bias_noise_ = acc_w;
    gyro_bias_noise_ = gyro_w;
    p_I_GNSS_ = p_IMU_GNSS;
    state_ptr_ = std::make_shared<State>();
    initialized_ = false;
}
bool ESKF::process_IMU_Data(IMUDataPtr imu_data_ptr)
{
    if(!initialized_)
    {
        imu_buffer_.push_back(imu_data_ptr);
        if(imu_buffer_.size() > IMU_Buffer_Size) imu_buffer_.pop_front();
        return false;
    }
    predict(last_imu_ptr_, imu_data_ptr);
    last_imu_ptr_ = imu_data_ptr;
    std::cout<<"Initialized and predict."<<std::endl;

    return true;
}

void ESKF::predict(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr)
{
    double dt = cur_imu_ptr->timestamp - last_imu_ptr->timestamp;
    double dt_2 = dt * dt;

    State last_state = *state_ptr_;

    // timestamp
    state_ptr_->timestamp = cur_imu_ptr->timestamp;
    // p v R
    Eigen::Vector3d acc_unbias = 0.5 * (last_imu_ptr->acc + cur_imu_ptr->acc) - last_state.acc_bias;
    Eigen::Vector3d gyr_unbias = 0.5 * (last_imu_ptr->gyro + cur_imu_ptr->gyro) - last_state.gyro_bias;
    Eigen::Vector3d acc_nominal = last_state.R_G_I * acc_unbias + Eigen::Vector3d(0., 0., -g);
    state_ptr_->p_G_I = last_state.p_G_I + last_state.v_G_I * dt + 0.5 * acc_nominal * dt_2;
    state_ptr_->v_G_I = last_state.v_G_I + acc_nominal * dt;
    Eigen::Vector3d delta_angle_axis = gyr_unbias * dt;
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if(delta_angle_axis.norm() > DBL_EPSILON)
    {
        dR = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
        state_ptr_->R_G_I = last_state.R_G_I * dR;
    }


    //Fx =
    //[I    I*dt    0    0    0]
    //[0    I -R[a]^dt -Rdt   0]
    //[0    0  Rt{w*dt}  0  Idt]
    //[0    0       0    I    0]
    //[0    0       0    0    I]

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -state_ptr_->R_G_I * SkewMatrix(acc_unbias) * dt;
    Fx.block<3, 3>(3, 9) = -state_ptr_->R_G_I * dt;
    if (delta_angle_axis.norm() > DBL_EPSILON) {
        Fx.block<3, 3>(6, 6) = dR.transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

    //Fi =
    //[0 0 0 0]
    //[I 0 0 0]
    //[0 I 0 0]
    //[0 0 I 0]
    //[0 0 0 I]

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();


    //Qi =
    //[dt_2*acc_n    0    0    0]
    //[0    dt_2*gyr_n    0    0]
    //[0     0      dt*acc_w   0]
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt_2 * acc_noise_;
    Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt_2 * gyro_noise_;
    Qi.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * dt * acc_bias_noise_;
    Qi.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt * gyro_bias_noise_;

    // P = Fx * P * Fxt + Fi * Q * Fit
    state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
}

bool ESKF::process_GNSS_Data(GNSSDataPtr gnss_data_ptr)
{
    if(!initialized_)
    {
        if(imu_buffer_.size() < IMU_Buffer_Size){
            std::cout<<"[ESKF] No enough IMU data."<<std::endl;
            return false;
        }
        last_imu_ptr_ = imu_buffer_.back();
        if(std::abs(last_imu_ptr_->timestamp - gnss_data_ptr->timestamp) > 0.2)
        {
            std::cout<<"[ESKF] GNSS and IMU are not sychonized."<<std::endl;
            return false;
        }
        bool ok = initialize();
        if(!ok) return false;
        init_lla_ = gnss_data_ptr->lla;
        initialized_ = true;
    }
    return true;
}

bool ESKF::initialize(void)
{
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();
    std::cout<<"[ESKF] Mean acc: "<<mean_acc[0]<<" "<<mean_acc[1]<<" "<<mean_acc[2]<<std::endl;

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (auto imu_data : imu_buffer_) sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();
    if (std_acc.maxCoeff() > IMU_Std) {
        std::cout<<"[ESKF] Big acc std: "<<std_acc[0]<<" "<<std_acc[1]<<" "<<std_acc[2]<<std::endl;
        return false;
    }
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d R_I_G;
    R_I_G.block<3, 1>(0, 0) = x_axis;
    R_I_G.block<3, 1>(0, 1) = y_axis;
    R_I_G.block<3, 1>(0, 2) = z_axis;

    state_ptr_->R_G_I = R_I_G.transpose();
    state_ptr_->timestamp = last_imu_ptr_->timestamp;
    state_ptr_->imu_data_ptr = last_imu_ptr_;
    state_ptr_->p_G_I.setZero();
    state_ptr_->v_G_I.setZero();
    state_ptr_->acc_bias.setZero();
    state_ptr_->gyro_bias.setZero();
    state_ptr_->cov.setZero();
    state_ptr_->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<2, 2>(6, 6) = 100. * D_R * D_R * Eigen::Matrix2d::Identity();
    state_ptr_->cov(8, 8) = 10000. * D_R * D_R;
    state_ptr_->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    state_ptr_->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();




    return true;
}


