#ifdef COMPILE_WITH_SLAM

#include "FSM/State_Slam.h"

State_Slam::State_Slam(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::SLAM;
    _stateNameString = "SLAM";
    initRecv();
}

FSMStateName State_Slam::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::SLAM;
    }
}

void State_Slam::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
    ros::spinOnce();
}

void State_Slam::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}

void State_Slam::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_Slam::twistCallback, this);
}

#endif  // COMPILE_WITH_SLAM