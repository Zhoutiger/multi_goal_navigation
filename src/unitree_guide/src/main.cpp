#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "interface/IOSDK.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_ROS
    #include "interface/KeyBoard.h"
    #include "interface/IOROS.h"
#endif  // COMPILE_WITH_ROS

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
	std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv){
    /* Settings */
    CtrlPlatform ctrlPlat = CtrlPlatform::GAZEBO_A1;  // 定义 是在Gazebo切换还是 实机
    // CtrlPlatform ctrlPlat = CtrlPlatform::REAL_A1;

    // set real-time process
    setProcessScheduler();
    // set the print format
    std::cout << std::fixed << std::setprecision(3);

    IOInterface *ioInter;
    if(ctrlPlat == CtrlPlatform::GAZEBO_A1){
#ifdef COMPILE_WITH_ROS
        ros::init(argc, argv, "unitree_gazebo_servo");
        ioInter = new IOROS();
#endif  // COMPILE_WITH_ROS
    }
    else if(ctrlPlat == CtrlPlatform::REAL_A1){
        ioInter = new IOSDK();
    }

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002;
    ctrlComp->running = &running;
    ctrlComp->robotModel = new A1Robot();

    //ctrlComp->waveGen = new WaveGenerator(1, 0.75, Vec4(0.25, 0.75, 0.5, 0));  
    ctrlComp->waveGen = new WaveGenerator(0.4, 0.5, Vec4(0, 0.5, 0.5, 0));  //Trot
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.55, Vec4(0, 0.5, 0.5, 0));  //Walking Trot
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0.5, 0, 0.5, 0));  //Pace
    //ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);   // 状态机

    signal(SIGINT, ShutDown);

    while(running){
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
