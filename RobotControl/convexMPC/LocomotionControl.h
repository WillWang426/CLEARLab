
#include "Controllers/FootSwingTrajectory.h"
#include "ControlData.h"
#include "cppTypes.h"
#include "Gait.h"

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;

class LocomotionControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocomotionControl(); 
  ~LocomotionControl();

  void initialize();

  void run(ControlData<float>& data);

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

private:
  void _SetupCommand(ControlData<float> & data);

  float _yaw_turn_rate = 0.;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_cmd = 0.25;
  float _y_vel_cmd = 0.;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  //float _body_height = 0.34;
  float _body_height = 0.29;

//   float _body_height_running = 0.29;
//   float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlData<float>& data, bool omniMode);
  void solveDenseMPC(int *mpcTable, ControlData<float> &data);
  void solveSparseMPC(int *mpcTable, ControlData<float> &data);
  void initSparseMPC();
  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
//   MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  float trajAll[12*36];

};