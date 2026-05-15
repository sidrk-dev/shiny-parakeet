#ifndef CALCPID_H
#define CALCPID_H

#include <stdint.h>
#include <math.h>

typedef struct { double kp, ki, kd, tmax, omax; } pidParam_t;

class CalcPID
{
public:
    void setParam(pidParam_t *pidParam_);
    double calc(double target_, double val_, uint32_t micros_);
    void reset();
private:
    pidParam_t *_pidParam = nullptr;
    struct { uint32_t curr, prev; double delta; } _t = {};
    struct { double curr, prev; } _deviation = {};
    struct { double p, i, d, o; } _value = {};
};

#endif