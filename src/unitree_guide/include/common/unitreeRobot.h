#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"
// #include "common/mathTypes.h"

class QuadrupedRobot{
public:
    QuadrupedRobot(){};
    ~QuadrupedRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec34 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // // Global Inverse Kinematics
    // Vec12 getQ(const Vec34 &feetPosition, HomoMat homoMat);
    // Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, HomoMat homoMat);
    // Vec12 getTau(const Vec12 &q, const Vec34 feetForce, HomoMat homoMat);

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);

    Mat3 getJaco(LowlevelState &state, int legID);
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    double getRobMass(){return _mass;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    QuadrupedLeg* _Legs[4];
    // Vec2 _footPlaceLimit;
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
    Vec34 _feetPosNormalStand;
    double _mass;
    Mat3 _Ib;
};

class A1Robot : public QuadrupedRobot{
public:
    A1Robot();
    ~A1Robot(){}
private:
    // Vec12 _qNormalStand;
    // LowlevelState _stateNormalStand;
    
};

#endif  // UNITREEROBOT_H