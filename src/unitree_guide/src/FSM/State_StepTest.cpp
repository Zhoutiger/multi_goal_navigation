#include "FSM/State_StepTest.h"

State_StepTest::State_StepTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::STEPTEST, "stepTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact),
                  _phase(ctrlComp->phase){

    _gaitHeight = 0.05;

    // _Kpp = Vec3(90, 90, 90).asDiagonal();
    // _Kpw = Vec3(200, 200, 200).asDiagonal();
    // _Kdp = Vec3(10, 10, 10).asDiagonal();
    // _Kdw = Vec3(20, 20, 20).asDiagonal();

    _KpSwing = Vec3(600, 600, 200).asDiagonal();
    _KdSwing = Vec3(20, 20, 5).asDiagonal();

    _Kpp = Vec3(50, 50, 300).asDiagonal();
    _Kpw = Vec3(600, 600, 600).asDiagonal();
    _Kdp = Vec3(5, 5, 20).asDiagonal();
    _Kdw = Vec3(10, 10, 10).asDiagonal();

    // _KpSwing = Vec3(100, 100, 150).asDiagonal();
    // _KdSwing = Vec3(10, 10, 15).asDiagonal();

    // for(int i(0); )

    // _ctrlComp->plot->addPlot("Feet goal Z", 4, std::vector<std::string>{"1", "2", "3", "4"});
    // _ctrlComp->plot->addPlot("Feet force Z", 4, std::vector<std::string>{"1", "2", "3", "4"});
    // _ctrlComp->plot->addPlot("dWbd", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("foot0 global force", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("positionXYZ", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("rotationXYZ", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("bodyVel", 3, std::vector<std::string>{"x", "y", "z"});
    // _ctrlComp->plot->addPlot("bodyPos", 3, std::vector<std::string>{"x", "y", "z"});
}

void State_StepTest::enter(){
    _pcd = _est->getPosition();
    _Rd  = _lowState->getRotMat();
    _posFeetGlobalInit = _est->getFeetPos();
    _posFeetGlobalGoal = _posFeetGlobalInit;
    _ctrlComp->setStartWave();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_StepTest::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();

// _ctrlComp->plot->addFrame("bodyVel", _velBody);
// _ctrlComp->plot->addFrame("bodyPos", _posBody);

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();


    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _posFeetGlobalGoal(2, i) = _posFeetGlobalInit(2, i) + (1-cos((*_phase)(i)*2*M_PI))*_gaitHeight;
            _velFeetGlobalGoal(2, i) = sin((*_phase)(i)*2*M_PI)*2*M_PI*_gaitHeight;
        }
    }

    calcTau();

    _lowCmd->setZeroGain();
    _lowCmd->setTau(_tau);
}

void State_StepTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
    // _ctrlComp->plot->showPlot("Feet force Z");
    // _ctrlComp->plot->showPlotAll();
}

FSMStateName State_StepTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::STEPTEST;
    }
}

void State_StepTest::calcTau(){
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _Kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

// _ctrlComp->plot->addFrame("positionXYZ", _pcd - _posBody);
// _ctrlComp->plot->addFrame("rotationXYZ", rotMatToExp(_Rd*_G2B_RotMat));
// _ctrlComp->plot->addFrame("dWbd", _dWbd);
    // _posFeetGlobal = _est->getFeetPos();
    // for(int i(0); i < 4; ++i){
    //     _posFeet2BGlobal.col(i) = _posFeetGlobal.col(i) - _posBody;
    // }

    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }
    
// _ctrlComp->plot->addFrame("foot0 global force", _forceFeetGlobal.col(0));

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

// _ctrlComp->plot->addFrame("Feet force Z", _forceFeetBody.row(2));
// _ctrlComp->plot->addFrame("Feet goal Z", _posFeetGlobalGoal.row(2));

// std::cout << "feet force Z: " << _forceFeetBody.row(2) << std::endl;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

}