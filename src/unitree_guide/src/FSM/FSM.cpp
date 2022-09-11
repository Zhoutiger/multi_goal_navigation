#include "FSM/FSM.h"
#include <iostream>


FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_SLAM
    _stateList.slam = new State_Slam(_ctrlComp);
#endif  // COMPILE_WITH_SLAM
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    // _ctrlComp->runAllEstTest();

    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        std::cout << "1" << std::endl;
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
        // _ctrlComp->_currentState = _nextStateName;
    }

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));// 等待
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    // case FSMStateName::EXIT:
    //     exit(0);
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_SLAM
    case FSMStateName::SLAM:
        return _stateList.slam;
        break;
#endif  // COMPILE_WITH_SLAM
    default:
        return _stateList.invalid;
        break;
    }
}

//偏航速度小于60度
bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}