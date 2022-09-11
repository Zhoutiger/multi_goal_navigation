#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#ifdef COMPILE_WITH_SLAM
    #include "FSM/State_Slam.h"
#endif  // COMPILE_WITH_SLAM

struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_FreeStand *freeStand;
    State_Trotting *trotting;
    State_BalanceTest *balanceTest;
    State_SwingTest *swingTest;
    State_StepTest *stepTest;
#ifdef COMPILE_WITH_SLAM
    State_Slam *slam;
#endif  // COMPILE_WITH_SLAM

    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete balanceTest;
        delete swingTest;
        delete stepTest;
#ifdef COMPILE_WITH_SLAM
        delete slam;
#endif  // COMPILE_WITH_SLAM
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};


#endif  // FSM_H