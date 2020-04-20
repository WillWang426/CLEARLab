
#include "RobotModel/include/RobotModel.hpp"
#include "StateEstimation/include/stateEstimation.h"

template <typename T>
struct ControlData {
  RobotModel<T>* _robotModel;
  StateEstimation<T>* _stateEstimator;
};

template struct ControlData<double>;
template struct ControlData<float>;
 
