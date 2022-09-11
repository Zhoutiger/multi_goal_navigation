#include "interface/WirelessHandle.h"
#include <string.h>
#include <stdio.h>

WirelessHandle::WirelessHandle(){
    // _L2Value = new LPFilter(0.002, 5.0);
    // _lxValue = new LPFilter(0.002, 5.0);
    // _lyValue = new LPFilter(0.002, 5.0);
    // _rxValue = new LPFilter(0.002, 5.0);
    // _ryValue = new LPFilter(0.002, 5.0);
}

void WirelessHandle::receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){
    memcpy(&_keyData, lowState->wirelessRemote, 40);

    if(((int)_keyData.btn.components.L2 == 1) && 
       ((int)_keyData.btn.components.B  == 1)){
        userCmd = UserCommand::L2_B;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L2_A;
    }
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L2_X;
    }

#ifdef COMPILE_WITH_SLAM
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.Y  == 1)){
        userCmd = UserCommand::L2_Y;
    }
#endif  // COMPILE_WITH_SLAM

    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L1_X;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L1_A;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.Y  == 1)){
        userCmd = UserCommand::L1_Y;
    }
    else if((int)_keyData.btn.components.start == 1){
        userCmd = UserCommand::START;
    }

    userValue.L2 = _keyData.L2;
    userValue.lx = _keyData.lx;
    userValue.ly = _keyData.ly;
    userValue.rx = _keyData.rx;
    userValue.ry = _keyData.ry;

    // _L2Value->addValue(_keyData.L2);
    // _lxValue->addValue(_keyData.lx);
    // _lyValue->addValue(_keyData.ly);
    // _rxValue->addValue(_keyData.rx);
    // _ryValue->addValue(_keyData.ry);

    // userValue.L2 = _L2Value->getValue();
    // userValue.lx = _lxValue->getValue();
    // userValue.ly = _lyValue->getValue();
    // userValue.rx = _rxValue->getValue();
    // userValue.ry = _ryValue->getValue();

    // std::cout << "lx: " << _keyData.lx << std::endl;
    // std::cout << "ly: " << _keyData.ly << std::endl;
    // std::cout << "rx: " << _keyData.rx << std::endl;
    // std::cout << "ry: " << _keyData.ry << std::endl;
    // std::cout << "************************" << std::endl;
}
