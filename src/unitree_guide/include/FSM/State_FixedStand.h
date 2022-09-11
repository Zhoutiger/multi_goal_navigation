#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H
#include <vector>
#include "FSM/FSMState.h"


#define PI 3.1415925

using namespace std;

static double fixedHeight = 0.3,fixedPitch = 0.1;
static int times = 0;
class Leg
{
public:
	Leg() :length({ 0.0838, 0.2, 0.2 }), theta({ 0, 0, 0 }) {}

	vector<double> length;
	vector<double> theta;
};


class RobotModel
{
public:
	RobotModel() { }
	RobotModel (double height, double pitch)
	{
		_height = height;
		_pitch = pitch;
		calcQ();
	}
	vector<double> getQ()
	{
		vector<double> ret;
		for (int i = 0; i < 4; i++)
		{
			ret.push_back(leg[i].theta[0]);
			ret.push_back(leg[i].theta[1]);
			ret.push_back(leg[i].theta[2]);
		}
		return ret;
	}

	void calcQ();
	//前腿i=0, 1 后退i=2, 3
	void calcQfromP(int i);

	Leg leg[4];
	double _height, _pitch;
	double _xlength = 0.1805*2;
};


class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    
private:
     float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
   // float _targetPos[12] = {0, 0.835809, -1.69886, 0, 0.835809 ,-1.69886,0, 0.584782, -1.14814, 0 ,0.584782 ,-1.14814};
    float _startPos[12];
    float _duration = 1000;   //steps
    float _percent = 0;       //%
};

#endif  // FIXEDSTAND_H