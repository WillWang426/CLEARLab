// this file contains some common upper datetypes
#ifndef CLEAR_DATA_TYPE
#define CLEAR_DATA_TYPE
#include "cppTypes.h"
struct SensorData
{
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4]; /* data */
};

struct CommandData
{
    /* data */
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];
};

struct IMUData {
    Vec3<float> accelerometer;
    Vec3<float> gyro;
    Quat<float> quat;
};

template <typename T>
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<T> contactEstimate;
    Vec3<T> position;
    Vec3<T> vBody;
    Quat<T> orientation;
    Vec3<T> omegaBody;
    RotMat<T> rBody;
    Vec3<T> rpy;

    Vec3<T> omegaWorld;
    Vec3<T> vWorld;
    Vec3<T> aBody, aWorld;

};




#endif