#ifdef COMPILE_WITH_SLAM

#ifndef STATE_SLAM_H
#define STATE_SLAM_H

#include "FSM/State_Trotting.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class State_Slam : public State_Trotting{
public:
    State_Slam(CtrlComponents *ctrlComp);
    ~State_Slam(){}
    FSMStateName checkChange();
private:
    void getUserCmd();
    void initRecv();
    void twistCallback(const geometry_msgs::Twist& msg);
    ros::NodeHandle _nm;
    ros::Subscriber _cmdSub;
    double _vx, _vy;
    double _wz;
};

#endif  // STATE_SLAM_H

#endif  // COMPILE_WITH_SLAM