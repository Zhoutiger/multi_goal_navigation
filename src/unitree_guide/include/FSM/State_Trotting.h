#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
    double dpitch = 0;
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    // void calcBalanceStiffness();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    //通过PD控制率求得的质心加速度和角加速度
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;
};

#endif  // TROTTING_H