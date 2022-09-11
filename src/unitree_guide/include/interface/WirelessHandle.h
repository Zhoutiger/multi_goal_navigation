#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "message/unitree_joystick.h"
#include "common/LowPassFilter.h"
#include "interface/CmdPanel.h"
#include "unitree_legged_sdk/comm.h"

class WirelessHandle : public CmdPanel{
public:
    WirelessHandle();
    ~WirelessHandle(){}
    void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState);
private:
    xRockerBtnDataStruct _keyData;
    // LPFilter *_L2Value, *_lxValue, *_lyValue, *_rxValue, *_ryValue;
};

#endif  // WIRELESSHANDLE_H