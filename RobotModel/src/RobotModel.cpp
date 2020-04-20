#include "RobotModel.h"

template <typename T>
void RobotData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();  
}

template <typename T>
void RobotCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

template <typename T>
void RobotModel<T>::zeroCommand() {
  for (auto& cmd : _robotCommand) {
    cmd.zero();
  }
}

template <typename T>
void RobotModel<T>::updateRobotData(SensorData* snsdata) {
  for (int leg = 0; leg < 4; leg++) {
    // q:
    _robotData[leg].q(0) = snsdata->q_abad[leg];
    _robotData[leg].q(1) = snsdata->q_hip[leg];
    _robotData[leg].q(2) = snsdata->q_knee[leg];

    // qd
    _robotData[leg].qd(0) = snsdata->qd_abad[leg];
    _robotData[leg].qd(1) = snsdata->qd_hip[leg];
    _robotData[leg].qd(2) = snsdata->qd_knee[leg];

    // _robotData[leg].J and _robotData[leg].p
    computeLegJacobianAndPosition(leg);

    // v
    _robotData[leg].v = _robotData[leg].J * _robotData[leg].qd;
  }
}

template <typename T>
void RobotModel<T>::updateFBMState(StateEstimate* stateEstimate) {
    FBModelState<T> state;
    state.bodyOrientation = _stateEstimate->orientation;
    state.bodyPosition    = _stateEstimate->position;
    state.bodyVelocity.head(3) = _stateEstimate->omegaBody;
    state.bodyVelocity.tail(3) = _stateEstimate->vBody;

    state.q.setZero(12);
    state.qd.setZero(12);

    for (int i = 0; i < 4; ++i) {
        state.q(3*i+0) = _robotData[i].q[0];
        state.q(3*i+1) = _robotData[i].q[1];
        state.q(3*i+2) = _robotData[i].q[2];
        state.qd(3*i+0)= _robotData[i].qd[0];
        state.qd(3*i+1)= _robotData[i].qd[1];
        state.qd(3*i+2)= _robotData[i].qd[2];
    }

    _model->setState(state);
}

template <typename T>
void RobotModel<T>::updateCommand(CommandData* cmdData) {
    for (int leg = 0; leg < 4; leg++) {
        Vec3<T> legTorque = _robotCommand[leg].tauFeedForward;

        // forceFF
        Vec3<T> footForce = _robotCommand[leg].forceFeedForward;

        // cartesian PD
        footForce +=
            _robotCommand[leg].kpCartesian * (_robotCommand[leg].pDes - _robotData[leg].p);
        footForce +=
            _robotCommand[leg].kdCartesian * (_robotCommand[leg].vDes - _robotData[leg].v);

        // Torque
        legTorque += _robotData[leg].J.transpose() * footForce;

        // set command:
        cmdData->tau_abad_ff[leg] = legTorque(0);
        cmdData->tau_hip_ff[leg] = legTorque(1);
        cmdData->tau_knee_ff[leg] = legTorque(2);

        // joint space pd
        // joint space PD
        cmdData->kd_abad[leg] = _robotCommand[leg].kdJoint(0, 0);
        cmdData->kd_hip[leg] = _robotCommand[leg].kdJoint(1, 1);
        cmdData->kd_knee[leg] = _robotCommand[leg].kdJoint(2, 2);

        cmdData->kp_abad[leg] = _robotCommand[leg].kpJoint(0, 0);
        cmdData->kp_hip[leg] = _robotCommand[leg].kpJoint(1, 1);
        cmdData->kp_knee[leg] = _robotCommand[leg].kpJoint(2, 2);

        cmdData->q_des_abad[leg] = _robotCommand[leg].qDes(0);
        cmdData->q_des_hip[leg] = _robotCommand[leg].qDes(1);
        cmdData->q_des_knee[leg] = _robotCommand[leg].qDes(2);

        cmdData->qd_des_abad[leg] = _robotCommand[leg].qdDes(0);
        cmdData->qd_des_hip[leg] = _robotCommand[leg].qdDes(1);
        cmdData->qd_des_knee[leg] = _robotCommand[leg].qdDes(2);

        // estimate torque
        _robotData[leg].tauEstimate =
            legTorque +
            _robotCommand[leg].kpJoint * (_robotCommand[leg].qDes - _robotData[leg].q) +
            _robotCommand[leg].kdJoint * (_robotCommand[leg].qdDes - _robotData[leg].qd);
    }
}

template <typename T>
void RobotModel<T>::computeLegJacobianAndPosition(int leg) {

    T l1 = _quadruped->_abadLinkLength;
    T l2 = _quadruped->_hipLinkLength;
    T l3 = _quadruped->_kneeLinkLength;
    T l4 = _quadruped->_kneeLinkY_offset;
    T sideSign = _quadruped->getSideSign(leg);

    T s1 = std::sin(q(0));
    T s2 = std::sin(q(1));
    T s3 = std::sin(q(2));

    T c1 = std::cos(q(0));
    T c2 = std::cos(q(1));
    T c3 = std::cos(q(2));

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    _robotData[leg].J->operator()(0, 0) = 0;
    _robotData[leg].J->operator()(0, 1) = l3 * c23 + l2 * c2;
    _robotData[leg].J->operator()(0, 2) = l3 * c23;
    _robotData[leg].J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    _robotData[leg].J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    _robotData[leg].J->operator()(1, 2) = -l3 * s1 * s23;
    _robotData[leg].J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    _robotData[leg].J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    _robotData[leg].J->operator()(2, 2) = l3 * c1 * s23;
    

    _robotData[leg].p->operator()(0) = l3 * s23 + l2 * s2;
    _robotData[leg].p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    _robotData[leg].p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
}

template struct RobotCommand<double>;
template struct RobotCommand<float>;

template struct RobotData<double>;
template struct RobotData<float>;

template class RobotModel<double>;
template class RobotModel<float>;


