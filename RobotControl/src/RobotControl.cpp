#include "RobotControl.hpp"

RobotControl::RobotControl(RobotModel<float>* robotModel,   StateEstimation<float>* stateEstimator) {
    controlData->_robotModel = robotModel;
    controlData->_stateEstimator = stateEstimator;
}

void RobotControl::runController() {
    locomotion.run(*controlData);
}