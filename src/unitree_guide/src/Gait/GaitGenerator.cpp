#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp);

    _firstRun = true;

    _contactPast.setOnes();

    // _testGaitPlot.addPlot("foot1 pos", 3, std::vector<std::string>{"x", "y", "z"});
}

GaitGenerator::~GaitGenerator(){
    // _testGaitPlot.showPlotAll();
}

//设置步态速度、角速度和高度
void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _waveG->restart();
    _firstRun = true;
    _vxyGoal.setZero();
}

//足底位置
void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){
        _contactPast.setOnes();
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    // _idealP = _est->getPosFeetIdeal();

    for(int i(0); i<4; ++i){
        //本周期为支撑足
        if((*_contact)(i) == 1){
            //上一周期为摆动足
            if(_contactPast(i) == 0){
                _startP.col(i) = _est->getFootPos(i);
                // _startP.col(i) = _pastP.col(i);
            }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        //本周期为摆动足
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }

// std::cout << "idealP: " << _idealP.col(0).transpose() << std::endl;
// std::cout << "Raibert: " << (_feetCal->funcRaibert(_vxyGoal)).transpose() << std::endl;
    // _testGaitPlot.addFrame("foot1 pos", feetPos.col(0));
    _pastP = feetPos;
    _contactPast = *_contact;
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

//X、Y方向足底轨迹
float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

//X、Y方向足底速度轨迹
float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

//Z方向足底轨迹
float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

//Z方向足底速度轨迹
float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}