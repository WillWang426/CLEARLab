#include "../include/stateEstimation.h"
//#include "RobotModel/include/RobotModel.h"
#include "Common/include/Math/orientation_tools.h"
using namespace ori;



template <typename T>
void StateEstimation<T>::setup() {
  T dt = 0.002;//
  _xhat.setZero();
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  _C.block(6, 0, 3, 6) = C1;
  _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();
  _C.block(12, 0, 3, 6) = C2;
  _C.block(15, 0, 3, 6) = C2;
  _C.block(18, 0, 3, 6) = C2;
  _C.block(21, 0, 3, 6) = C2;
  _C(27, 17) = T(1);
  _C(26, 14) = T(1);
  _C(25, 11) = T(1);
  _C(24, 8) = T(1);
  _P.setIdentity();
  _P = T(100) * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
  _R0.setIdentity();
}


template <typename T>
void StateEstimation<T>::orientationEstimator() {
    this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.xsensData->quat[3];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.xsensData->quat[0];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.xsensData->quat[1];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.xsensData->quat[2];

    if(_b_first_visit){
        Vec3<T> rpy_ini = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
        rpy_ini[0] = 0;
        rpy_ini[1] = 0;
        _ori_ini_inv = rpyToQuat(-rpy_ini);
        _b_first_visit = false;
    }
    this->_stateEstimatorData.result->orientation =
            ori::quatProduct(_ori_ini_inv, this->_stateEstimatorData.result->orientation);

    this->_stateEstimatorData.result->rpy =
            ori::quatToRPY(this->_stateEstimatorData.result->orientation);


    this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
            this->_stateEstimatorData.result->orientation);

    this->_stateEstimatorData.result->omegaBody =
            this->_stateEstimatorData.xsensData->gyro.template cast<T>();

    this->_stateEstimatorData.result->omegaWorld =
            this->_stateEstimatorData.result->rBody.transpose() *
            this->_stateEstimatorData.result->omegaBody;

    this->_stateEstimatorData.result->aBody =
            this->_stateEstimatorData.xsensData->accelerometer.template cast<T>();
    this->_stateEstimatorData.result->aWorld =
            this->_stateEstimatorData.result->rBody.transpose() *
            this->_stateEstimatorData.result->aBody;
}

template <typename T>
void StateEstimation<T>::contactEstimator() {
  this->_stateEstimatorData.result->contactEstimate =
        *this->_stateEstimatorData.contactPhase;
}


template <typename T>
void StateEstimation<T>::linearKFPositionVelocityEstimator() {
  
  
  
  T process_noise_pimu =0.02;
  T process_noise_vimu =0.02;
  T process_noise_pfoot = 0.002;
  T sensor_noise_pimu_rel_foot = 0.001;
  T sensor_noise_vimu_rel_foot = 0.1;
  T sensor_noise_zfoot = 0.001;

  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  Vec3<T> g(0, 0, T(-9.81));
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  
  Vec3<T> a = this->_stateEstimatorData.result->aWorld + g; 
  
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;
 
    Vec3<T> ph = _model->_quadruped.getHipLocation(i);  // hip positions relative to CoM
 
    Vec3<T> p_rel = ph + this->_model->_robotData[i].p;
    
    Vec3<T> dp_rel = this->_model->_robotData[i].v;  
    Vec3<T> p_f = Rbod * p_rel;
    Vec3<T> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    T trust = T(1);
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));
    //T trust_window = T(0.25);
    T trust_window = T(0.2);

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (T(1) - trust_window)) {
      trust = (T(1) - phase) / trust_window;
    }
    //T high_suspect_number(1000);
    T high_suspect_number(100);

    // printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}

template <typename T>
void StateEstimation<T>::run(){
  
  this->orientationEstimator();
  this->contactEstimator();
  this->linearKFPositionVelocityEstimator();
}


template class StateEstimation<float>;
template class StateEstimation<double>;