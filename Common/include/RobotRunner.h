/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H


// #include "Controller/DesiredStateCommand.h"
#include "Controller/LegController.h"
#include "Dynamics/Quadruped.h"
#include "RobotController.h"
#include <lcm-cpp.hpp>
#include "cppTypes.h"

class RobotRunner{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController* robot_ctrl, RobotType robot_type){
    _robot_ctrl = robot_ctrl;
    _robotType = robot_type;
  }
  void init();
  void run();
  void cleanup();

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator(bool cheaterMode = false);
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;
  RobotType _robotType;
  // SpiData* spiData;
  // SpiCommand* spiCommand;

 private:
  
  int iter = 0;

  void setupStep();
  void finalizeStep();

  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  // DesiredStateCommand<float>* _desiredStateCommand;
  // rc_control_settings rc_control;
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;
  // state_estimator_lcmt state_estimator_lcm;
  // leg_control_data_lcmt leg_control_data_lcm;
  

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
