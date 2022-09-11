#include "FSM/State_BalanceTest.h"

State_BalanceTest::State_BalanceTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::BALANCETEST, "balanceTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    // _Kpp = Vec3(20, 20, 150).asDiagonal();
    // _Kdp = Vec3(9, 9, 25).asDiagonal();

    _Kpp = Vec3(150, 150, 150).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200;
    _Kdw = Vec3(30, 30, 30).asDiagonal();

    // _ctrlComp->plot->addPlot("_posBody", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("eular", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("tau", 12);
    // _ctrlComp->plot->addPlot("_ddPcd", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("_dWbd", 3, std::vector<std::string>{"x", "y", "z"});
}

void State_BalanceTest::enter(){
    _pcdInit = _est->getPosition();
    _pcd = _pcdInit;
    _RdInit = _lowState->getRotMat();

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_BalanceTest::run(){
    _userValue = _lowState->userValue;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.ly, _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue.lx, _yMin, _yMax);
    //_pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);
    _pcd(2) = _ctrlComp->ioInter->balanced_stand_height;
    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

    RotMat tmp;
    double s2 = sin(_ctrlComp->ioInter->balanced_stand_pitch);
    double c2 = cos(_ctrlComp->ioInter->balanced_stand_pitch);
    tmp << c2,0,-s2,0,1,0,s2,0,c2;
    _B2G_RotMat = tmp*_lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();

// std::cout << "tau: " << _tau.transpose() << std::endl;
// _ctrlComp->plot->addFrame("tau", _tau);

    _lowCmd->setZeroGain();
    _lowCmd->setTau(_tau);
}

void State_BalanceTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();

// _ctrlComp->plot->showPlotAll();

}

FSMStateName State_BalanceTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::BALANCETEST;
    }
}

void State_BalanceTest::calcTau(){
// _ctrlComp->plot->addFrame("_posBody", _posBody);
// _ctrlComp->plot->addFrame("eular", rotMatToExp(_Rd*_G2B_RotMat));

    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

// _ctrlComp->plot->addFrame("_ddPcd", _ddPcd);
// _ctrlComp->plot->addFrame("_dWbd", _dWbd);

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}