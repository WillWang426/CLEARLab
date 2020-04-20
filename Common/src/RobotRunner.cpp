/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>
#include "RobotRunner.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "Utilities/Timer.h"


/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);

  // memset(&rc_control, 0, sizeof(rc_control_settings));
  // Initialize the DesiredStateCommand object
  // _desiredStateCommand =
  //   new DesiredStateCommand<float>(driverCommand,
  //       &rc_control,
  //       controlParameters,
  //       &_stateEstimate,
  //       controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  // _robot_ctrl->_stateEstimator = _stateEstimator;
  // _robot_ctrl->_stateEstimate = &_stateEstimate;
  
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  // _stateEstimator->run();
  
  // Update the data from the robot
  setupStep();

  _robot_ctrl->runController();
        
  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  _legController->updateData(spiData);// ToDo

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep(){
  //update leg control command
  _legController->updateCommand(spiCommand);
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  // _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  // _lcm.publish("state_estimator", &state_estimator_lcm);
  _iterations++;
}

RobotRunner::~RobotRunner() {
  delete _legController;
  // delete _stateEstimator;
  // delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
