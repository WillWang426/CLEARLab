
// #include <main_helper.h>
#include "RobotModel.hpp"
#include "RobotControl.hpp"
#include "StateEstimation/include/stateEstimation.h"
// #include "cppTypes.h"
#include "ros/ros.h"

// Leg data from laikago ros
struct JointData
{
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
};


void jointDataCallback() // TODO


int main(int argc, char **argv) {
  
  ros::init(argc,argv,"communication")

  ros::NodeHandle nh;

  ros::Publisher jointCommandPublis = nh.advertise<>(); //TODO

  ros::Subscriber jointStateSubscriber = nh.subscribe(); //TODO

  IMUData* ROSIMU = nullptr;//TODO

  RobotModel<float>* robotModel = new RobotModel();
  StateEstimation<float>* stateEstimator = new StateEstimation(robotModel,ROSIMU); //TODO

  RobotControl* robotControl = new RobotControl(robotModel,stateEstimator);

  while (ros::ok())
  {
    // update robot data and zero robot command
    robotModel->updateRobotData(snsdata); 
    stateEstimator->run(); //TODO
    robotModel->updateFBMState(stateEstimator); //TODO
    robotModel->zeroCommand();

    // run robot control
    robotControl->runController();

    // update robot command
    robotModel->updateRobotCommand(cmdToSend);
    jointCommandPublis.publish(); // TODO

  }
  

}
