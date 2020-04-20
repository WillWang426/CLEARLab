//this file contains all the state estimation algorithm
#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H
#include "CLEAR_Datetype.h"
//#include "RobotModel/include/RobotModel.h"
template <typename T>
class RobotModel;
//some struct needed
struct IMUData {
    Vec3<float> accelerometer;
    Vec3<float> gyro;
    Quat<float> quat;
};

template <typename T>
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec4<T> contactEstimate;
    Vec3<T> position;
    Vec3<T> vBody;
    Quat<T> orientation;
    Vec3<T> omegaBody;
    RotMat<T> rBody;
    Vec3<T> rpy;

    Vec3<T> omegaWorld;
    Vec3<T> vWorld;
    Vec3<T> aBody, aWorld;

};

template <typename T>
struct StateEstimatorData {
    StateEstimate<T>* result;  
    IMUData* xsensData;
    //LegControllerData<T>* legControllerData;
    //RobotData<T>* legData;
    Vec4<T>* contactPhase;
    
};



template <typename T>
class StateEstimation{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    StateEstimation(RobotModel<T>* model, IMUData* imudata){
        _model = model;
        _stateEstimatorData.xsensData = imudata;
        this->setup();
    }
    
    ~StateEstimation(){
    delete _model;
    }

    void orientationEstimator();
    void contactEstimator();
    void linearKFPositionVelocityEstimator();
    void setup();
    void run();
//
//    void run();
// get results
    const StateEstimate<T>& getResult() { 
        return *_stateEstimatorData.result; }

     void setContactPhase(Vec4<T>& phase) { 
    *_stateEstimatorData.contactPhase = phase; 
  }

    StateEstimatorData<T> _stateEstimatorData;
    RobotModel<T>* _model = nullptr;
    
//member
private:
    StateEstimatorData<T> _data;
    Vec4<T> _phase;

//  velocity and position estimation parameters
    Eigen::Matrix<T, 18, 1> _xhat;
    Eigen::Matrix<T, 12, 1> _ps;
    Eigen::Matrix<T, 12, 1> _vs;
    Eigen::Matrix<T, 18, 18> _A;
    Eigen::Matrix<T, 18, 18> _Q0;
    Eigen::Matrix<T, 18, 18> _P;
    Eigen::Matrix<T, 28, 28> _R0;
    Eigen::Matrix<T, 18, 3> _B;
    Eigen::Matrix<T, 28, 18> _C;


protected:
    bool _b_first_visit = true;
    Quat<T> _ori_ini_inv;

};
template struct StateEstimate<float>;
template struct StateEstimate<double>;
template struct StateEstimatorData<float>;
template struct StateEstimatorData<double>;


#endif