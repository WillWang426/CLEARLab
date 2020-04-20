#include "Dynamics/Quadruped.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"

template <typename T>
struct RobotData
{   
    RobotData()
    {
        zero();
    }

    void zero();
    Vec3<T> q, qd, p, v;
    Mat3<T> J;
};

template <typename T>
struct RobotCommand
{
    void zero();
    Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
    Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

template <typename T>
class RobotModel
{
public:
    RobotModel(){
        _quadruped = buildMiniCheetah<T>();
        _model = _quadruped->buildModel<T>();
    };
    
    ~RobotModel(){
        delete _quadruped;
    };

    Quadruped<T>* _quadruped = nullptr;
    FloatingBaseModel<T> _model;
    RobotData<T> _robotData[4];
    RobotCommand<T> _robotCommand[4];

    void zeroCommand(); 
    void updateRobotData(SensorData* snsdata); //TODO
    void updateFBMState(StateEstimate* stateEstimate); //TODO
    void updateCommand(CommandData* cmdData); //TODO

private:
    void computeLegJacobianAndPosition(int leg);
};


