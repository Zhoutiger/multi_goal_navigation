#ifndef CMDPANEL_H
#define CMDPANEL_H

#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <pthread.h>

struct UserValue{
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue(){
        setZero();
    }
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
};

class CmdPanel{
public:
    CmdPanel(){}
    ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    void setPassive(){userCmd = UserCommand::L2_B;}
    //控制指令设置为0
    void setZero(){userValue.setZero();}
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};

    UserCommand userCmd;
    UserValue userValue;
    
protected:
    virtual void *run(void *arg){};

};

#endif  // CMDPANEL_H