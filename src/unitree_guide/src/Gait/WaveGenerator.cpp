#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
              :_period(period), _stRatio(stancePhaseRatio), _bias(bias){
    _start = false;

    if((_stRatio >= 1) || (_stRatio <= 0)){
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for(int i(0); i < bias.rows(); ++i){
        if((bias(i) > 1) || (bias(i) < 0)){
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    // _testPlot.addPlot("wave", 4, std::vector<std::string>{"foot1", "foot2", "foot3", "foot4"});
    // _testPlot.addPlot("contact", 4, std::vector<std::string>{"foot1", "foot2", "foot3", "foot4"});
}

WaveGenerator::~WaveGenerator(){
    // _testPlot.printXY("wave", "foot1", -1, 5);
    // _testPlot.showPlotAll();
}

void WaveGenerator::calWave(Vec4 &phase, VecInt4 &contact){
    if(_start == false){
        _start = true;
        _startT = getSystemTime();
    }

    _passT = (double)(getSystemTime() - _startT) * 1e-6;
    for(int i(0); i < 4; ++i){
        //计算相位进度
        _normalT(i) = fmod(_passT + _period - _period*_bias(i), _period) / _period;
        //_normalT(i) = fmod(_passT + _period - _period*_bias(i), _period);
        // _normalT(i) = fmod(_passT + _period*(1+_bias(i)*_stRatio), _period) / _period;
        // _normalT(i) = fmod(_normalT(i), 1);

        //判断是支撑足还是摆动足
        if(_normalT(i) < _stRatio){
            //是否为支撑足
            contact(i) = 1;
            //支撑或摆动相位进度
            phase(i) = _normalT(i) / _stRatio;
            _feetStart(i) = 1;
        }else{
            phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            if(_feetStart(i) == 1){
                contact(i) = 0;
            }else{
                contact(i) = 1;
            }
        }
    }

    // _testPlot.addFrame("wave", _passT, phase);
    // _testPlot.addFrame("contact", _passT, contact);
}

//每个周期支撑时长
float WaveGenerator::getTstance(){
    return _period * _stRatio;
}

//每个摆动时长
float WaveGenerator::getTswing(){
    return _period * (1 - _stRatio);
}

//周期时长
float WaveGenerator::getT(){
    return _period;
}

void WaveGenerator::restart(){
    _start = false;
}