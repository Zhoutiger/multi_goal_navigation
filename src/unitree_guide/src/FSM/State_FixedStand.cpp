#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){
                    
    }

void State_FixedStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO_A1){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REAL_A1){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();
   
}

void State_FixedStand::run(){
    //  times++;
    // RobotModel rm(fixedHeight, fixedPitch);
    //  vector<double> ret = rm.getQ();
    //   for(int i = 0; i < 12; i++)
    // {
    //     _targetPos[i] = ret[i];
    //     cout << -_targetPos[i]<< endl;
    // }
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
       _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
        //_lowCmd->motorCmd[j].q = _targetPos[j]; 
    }
}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_X){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_A){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::L1_Y){
        return FSMStateName::STEPTEST;
    }
#ifdef COMPILE_WITH_SLAM
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::SLAM;
    }
#endif  // COMPILE_WITH_SLAM
    else{
        return FSMStateName::FIXEDSTAND;
    }
}

void RobotModel::calcQ()
{
	for (int i = 0; i < 4; i++)
		calcQfromP(i);
}

void RobotModel::calcQfromP(int i)
{
	double deltaX = _xlength / 2 - _xlength * cos(_pitch) / 2;
	double deltaZ = _xlength * sin(_pitch) / 2;
	double x = 0, z = _height;
	vector<double> ret;
	if (i == 0 || i == 1)
	{
		x = deltaX;
		z = _height + deltaZ;
	}
	else if (i == 2 || i == 3)
	{
		x = -deltaX;
		z = _height - deltaZ;
	}
	double l1 = leg[i].length[1], l2 = leg[i].length[2];
	//cout << l1;
	double c3 = (x * x + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2);
	leg[i].theta[2] = -acos(c3);
	double tmp1 = atan2(z, x);
	double tmp2 = (x * x + z * z + l1 * l1 - l2 * l2) / (2 * l1 * pow(x * x + z * z, 0.5));
	leg[i].theta[1] = tmp1 + acos(tmp2) - PI/2;
}