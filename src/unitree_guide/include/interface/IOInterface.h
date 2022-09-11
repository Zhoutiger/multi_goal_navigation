#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){}
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}
int ppcd = 0.2;
float walk_height = 0.3, walk_pitch = 0, balanced_stand_height = 0.25, balanced_stand_pitch = 0;
float pre_walk_pitch = 0, pre_balanced_stand_pitch = 0;
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H