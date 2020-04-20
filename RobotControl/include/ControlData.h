#ifndef ROBOT_CONTROL
#define ROBOT_CONTROL
#include "RobotModel/include/RobotModel.h"
#include "StateEstimation/include/stateEstimation.h"

template <typename T>
struct ControlData {
  RobotModel<T>* _robotModel;
  StateEstimation<T>* _stateEstimator;
};

template struct ControlData<double>;
template struct ControlData<float>;
 
#endif