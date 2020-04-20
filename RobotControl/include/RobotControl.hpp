#include "StateEstimation/include/stateEstimation.h"
#include "ControlData.h"
#include "convexMPC/LocomotionControl.h"


class RobotControl {
public:
    RobotControl(RobotModel<float>* robotModel,StateEstimation<float>* stateEstimator);
    ~RobotControl(){}

    void runController();

private:
    ControlData<float>* controlData;
    LocomotionControl locomotion;
};