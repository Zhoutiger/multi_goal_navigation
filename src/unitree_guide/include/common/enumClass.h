#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO_A1,
    REAL_A1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // trotting
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
#ifdef COMPILE_WITH_SLAM
    L2_Y,       // slam
#endif  // COMPILE_WITH_SLAM
    L1_X,       // balanceTest
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
#ifdef COMPILE_WITH_SLAM
    SLAM,       // slam
#endif  // COMPILE_WITH_SLAM
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

// enum class WaveState{
//     WAVE,
//     ALLSTANCE,
//     ALLSWING
// };

// inline std::ostream & operator<<(std::ostream &os,const FSMStateName &ec)
//  {
//    os<<static_cast<std::underlying_type<FSMStateName>::type>(ec);
//    return os;
//  }

#endif  // ENUMCLASS_H