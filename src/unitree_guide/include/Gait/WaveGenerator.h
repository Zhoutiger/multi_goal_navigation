#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H


#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "common/enumClass.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
class WaveGenerator{
public:
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
    ~WaveGenerator();
    void calWave(Vec4 &phase, VecInt4 &contact);
    float getTstance();
    float getTswing();
    float getT();
    void restart();
private:
    double _period;
    double _stRatio;
    Vec4 _bias;

    Vec4 _normalT;                   // [0, 1)
    // VecInt4 _contact;
    bool _start;
    VecInt4 _feetStart;
    double _passT;                   // unit: second
    long long _startT;    // unit: us
#ifdef COMPILE_DEBUG
    PyPlot _testPlot;
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H