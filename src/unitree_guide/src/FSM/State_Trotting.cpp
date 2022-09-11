#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;
    //_gaitHeight = 0.13;

    //加速度PD控制
    _Kpp = Vec3(50, 50, 150).asDiagonal();
    _Kdp = Vec3(14, 14, 25).asDiagonal();

    //_kpw = 200;
    //_Kdw = Vec3(30, 30, 30).asDiagonal();

    //角加速度PD控制
    _kpw = 300;
    _Kdw = Vec3(20, 20, 20).asDiagonal();

    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(35, 35, 35).asDiagonal();

    _KpSwing = Vec3(300, 300, 300).asDiagonal();
    _KdSwing = Vec3(25, 25, 25).asDiagonal();

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    // _ctrlComp->plot->addPlot("foot1_PosGoal", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot1_VelGoal", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot1_Pos", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot1_Vel", 3, std::vector<std::string>{"x", "y", "z"});

    // _ctrlComp->plot->addPlot("bodyPos", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("errorVel", 3, std::vector<std::string>{"x", "y", "z"});

    // _ctrlComp->plot->addPlot("bodyVel", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot0Pos", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot0PosGoal", 3, std::vector<std::string>{"x", "y", "z"});

    // _ctrlComp->plot->addPlot("foot0PosError", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot0VelError", 3, std::vector<std::string>{"x", "y", "z"});
}


State_Trotting::~State_Trotting(){
    delete _gait;
}

void State_Trotting::enter(){
    _pcd = _est->getPosition();
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
    _ctrlComp->setStartWave();
}

void State_Trotting::exit(){
    // _ctrlComp->plot->showPlotAll();
    
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    RotMat tmp;
    double s2 = sin(_ctrlComp->ioInter->walk_pitch);
    double c2 = cos(_ctrlComp->ioInter->walk_pitch);
    tmp << c2,0,-s2,0,1,0,s2,0,c2;
    _B2G_RotMat = tmp*_lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

// _ctrlComp->plot->addFrame("foot0Pos", _G2B_RotMat*_posFeet2BGlobal.col(0));
// _ctrlComp->plot->addFrame("bodyPos", _G2B_RotMat*_posBody);
// _ctrlComp->plot->addFrame("bodyVel", _G2B_RotMat*_velBody);

    _userValue = _lowState->userValue;

    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

// _ctrlComp->plot->addFrame("foot0PosGoal", _G2B_RotMat*(_posFeetGlobalGoal.col(0) - _posBody));

// _ctrlComp->plot->addFrame("foot1_PosGoal", _posFeetGlobalGoal.col(0));
// _ctrlComp->plot->addFrame("foot1_VelGoal", _velFeetGlobalGoal.col(0));

    calcTau();
    calcQQd();

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        // _lowCmd->setQ(i, _qGoal.col(i));
        // _lowCmd->setQd(i, _qdGoal.col(i));
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    //_vCmdBody(1) = -invNormalize(_userValue.rx, _vyLim(0), _vyLim(1));
    _vCmdBody(1) = invNormalize(_userValue.rx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    //_dYawCmd = -invNormalize(_userValue.lx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = invNormalize(_userValue.lx, _wyawLim(0), _wyawLim(1));
}

//计算质心期望位姿
void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
    _vCmdGlobal(2) = 0;


    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
    _pcd(2) = _ctrlComp->ioInter->walk_height;
    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd) * I3;
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Trotting::calcTau(){// zen me  kan 
    //PD控制
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (_vCmdGlobal - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());
    //通过平衡控制得到的足端力
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

// _ctrlComp->plot->addFrame("foot1_Pos", _posFeetGlobal.col(0));
// _ctrlComp->plot->addFrame("foot1_Vel", _velFeetGlobal.col(0));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody);
        // _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _vCmdGlobal);
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2BGoal, _velFeet2BGoal, FrameType::BODY));
}
