#include "CalcPID.h"

void CalcPID::setParam(pidParam_t *pidParam_) {
    _pidParam = pidParam_;
}

double CalcPID::calc(double target_, double val_, uint32_t micros_) {
    if (_pidParam == nullptr) return 0.0;
    if ((_pidParam->tmax <= 0.0) || (_pidParam->omax <= 0.0)) return 0.0;
    if (target_ > _pidParam->tmax) target_ = _pidParam->tmax;
    if (target_ < -_pidParam->tmax) target_ = -_pidParam->tmax;
    if ((_pidParam->kp == 0.0) && (_pidParam->ki == 0.0) && (_pidParam ->kd == 0.0)) return target_ * (_pidParam->omax / _pidParam->tmax);
    _deviation.curr = target_ - val_;
    _t.curr = micros_;
    if (_t.prev == 0) {
        _t.prev = _t.curr;
        _deviation.prev = _deviation.curr;
        _value = { 0.0, 0.0, 0.0, 0.0 };
        return 0.0;
    }
    uint32_t diff = _t.curr - _t.prev;
    _t.delta = (double)diff / 1000000.0;
    if (_t.delta <= 0.0) {
        _t.prev = _t.curr;
        _deviation.prev = _deviation.curr;
        return _value.o;
    }
    _t.prev = _t.curr;
    _value.p = _pidParam->kp * _deviation.curr;
    _value.i += _pidParam->ki * (_deviation.curr + _deviation.prev) * _t.delta / 2.0;
    _value.d = _pidParam->kd * (_deviation.curr - _deviation.prev) / _t.delta;
    _value.o = _value.p + _value.i + _value.d;
    if (_value.o > _pidParam->omax) _value.i = _pidParam->omax - (_value.p + _value.d);
    if (_value.o < -_pidParam->omax) _value.i = -_pidParam->omax - (_value.p + _value.d);
    _value.o = _value.p + _value.i + _value.d;
    _deviation.prev = _deviation.curr;
    return _value.o;
}

void CalcPID::reset() {
    _t = { 0, 0, 0.0 };
    _deviation = { 0.0, 0.0 };
    _value = { 0.0, 0.0, 0.0, 0.0 };
}
