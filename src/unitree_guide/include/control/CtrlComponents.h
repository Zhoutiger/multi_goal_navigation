#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H


#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
double a = 0.25;
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
        // estTestNum = sizeof(estTest)/sizeof(estTest[0]);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;

#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    // Estimator* estTest[3];
    // unsigned int estTestNum;
    BalanceCtrl *balCtrl;

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG
    // FSMStateName _currentState = FSMStateName::PASSIVE;

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    bool *running;
    bool startGait = false;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

//四条腿的状态
    void runWaveGen(){
        if(startGait){
            waveGen->calWave(*phase, *contact);
        }else{
            waveGen->restart();
        }
    }

//所有足支撑
    void setAllStance(){
        startGait = false;
        contact->setOnes();
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }

//所有足摆动
    void setAllSwing(){
        startGait = false;
        contact->setZero();
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }

    void setStartWave(){
        startGait = true;
    }

    // void startWaveGene(){

    // }

    // void endWaveGene(){

    // }

//创建状态估计器和平衡器对象
    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        estimator->setPyPlot(plot);
#endif  // COMPILE_DEBUG
    }

};

#endif  // CTRLCOMPONENTS_H