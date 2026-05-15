#include "RoboMaster.h"

template <class Traits>
void Driver<Traits>::setID(uint8_t id_) {
    if ((id_ < 1) || (8 < id_)) return;
    _data.id = id_;
}

template <class Traits>
void Driver<Traits>::setCurrent(struct can_frame *sendMsg_, double current_) {
    if (_data.id == 0) return;
    if (Traits::currentRangeA < current_) current_ = Traits::currentRangeA;
    if (current_ < -Traits::currentRangeA) current_ = -Traits::currentRangeA;
    int16_t encodedCurrent = (int16_t)((double)Traits::currentResolution * (current_ / Traits::currentRangeA));
    bytes_t bytes = _divide(encodedCurrent);
    sendMsg_->data[((_data.id - 1) % 4) * 2] = bytes.upper;
    sendMsg_->data[((_data.id - 1) % 4) * 2 + 1] = bytes.lower;
}

template <class Traits>
void Driver<Traits>::update(const struct can_frame &readMsg_) {
  if (_data.id == 0) return;
  if (readMsg_.can_id != (0x200 + _data.id)) return;
  bytes_t bytes[3];
  for (uint8_t i = 0; i < 3; i++) {
    bytes[i].upper = readMsg_.data[2 * i];
    bytes[i].lower = readMsg_.data[2 * i + 1];
  }
  _data.angle = (360.0 / (double)Traits::encoderResolution) * (double)_connect(bytes[0]);
  _data.speed = _connect(bytes[1]);
  _data.current = (Traits::currentRangeA / Traits::currentResolution) * (double)_connect(bytes[2]);
  _data.temperature = readMsg_.data[6];
}

template <class Traits>
typename Driver<Traits>::driverData_t Driver<Traits>::getData() {
    return _data;
}

template <class Traits>
typename Driver<Traits>::bytes_t Driver<Traits>::_divide(int16_t val_) {
    uint16_t value = static_cast<uint16_t>(val_);
    bytes_t bytes;
    bytes.upper = static_cast<uint8_t>((value >> 8) & 0xFF);
    bytes.lower = static_cast<uint8_t>((value & 0xff));
    return bytes;
}

template <class Traits>
int16_t Driver<Traits>::_connect(bytes_t bytes_) {
    uint16_t value = (static_cast<uint16_t>(bytes_.upper) << 8) | static_cast<uint16_t>(bytes_.lower);
    return static_cast<int16_t>(value);
}

template <class Traits>
void Motor<Traits>::setPidInterval(uint8_t angle_, uint8_t speed_, uint8_t current_) {
    _interval.angle = angle_;
    _interval.speed = speed_;
    _interval.current = current_;
}

template <class Traits>
void Motor<Traits>::init(uint8_t id_) {
    _angle = {};
    _speed = {};
    _current = {};
    _angleRawprev = 0.0;
    _angleError = 0.0;
    _driver.setID(id_);
    _calcpid.angle.setParam(&pidParam.angle);
    _calcpid.speed.setParam(&pidParam.speed);
    _calcpid.current.setParam(&pidParam.current);
    _refreshCount = 0;
    _driverData = _driver.getData();
}

template <class Traits>
bool Motor<Traits>::refresh(uint32_t micros_, struct can_frame (&sendMsg_)[2], const struct can_frame &readMsg_) {
    if (readMsg_.can_id != (0x200 + _driverData.id)) return false;
    if (mode == MODE::UNUSED) return false;
    _refreshCount++;
    struct { bool angle, speed, current; } flag = {};
    if ((_refreshCount % _interval.angle) == 0) flag.angle = true;
    if ((_refreshCount % _interval.speed) == 0) flag.speed = true;
    if ((_refreshCount % _interval.current) == 0) flag.current = true;
    _driver.update(readMsg_);
    _read();
    if (flag.angle && (mode == MODE::ANGLE)) target.speed = _calcpid.angle.calc(target.angle, _angle.scaled, micros_);
    if (flag.speed && ((mode == MODE::ANGLE) || (mode == MODE::SPEED))) target.current = _calcpid.speed.calc(target.speed, _speed.scaled, micros_);
    if (flag.current && ((mode == MODE::ANGLE) || (mode == MODE::SPEED)|| (mode == MODE::CURRENT))) {
        target.output = _calcpid.current.calc(target.current, _current.scaled, micros_);
        if ((1 <= _driverData.id) && (_driverData.id <= 4)) {
            sendMsg_[0].can_id = CAN_1_TO_4_ID;
            sendMsg_[0].can_dlc = 8;
            _driver.setCurrent(&sendMsg_[0], target.output * static_cast<double>(direction));
        }
        if ((5 <= _driverData.id) && (_driverData.id <= 8)) {
            sendMsg_[1].can_id = CAN_5_TO_8_ID;
            sendMsg_[1].can_dlc = 8;
            _driver.setCurrent(&sendMsg_[1], target.output * static_cast<double>(direction));
        }
    }
    return true;
}

template <class Traits>
double Motor<Traits>::getAngle() {
    return _angle.scaled;
}

template <class Traits>
double Motor<Traits>::getSpeed() {
    return _speed.scaled;
}

template <class Traits>
double Motor<Traits>::getCurrent() {
    return _current.scaled;
}

template <class Traits>
void Motor<Traits>::resetIntegral() {
    _calcpid.angle.reset();
    _calcpid.speed.reset();
    _calcpid.current.reset();
}

template <class Traits>
void Motor<Traits>::resetAngle(double currAngle_) {
    double base = static_cast<double>(direction) * _angle.raw / (static_cast<double>(gearRatio) * Traits::innerGearRatio);
    _angleError = base - currAngle_;
}

template <class Traits>
double Motor<Traits>::_calcAngleError(double angleRawCurr_) {
    double error = angleRawCurr_ - _angleRawprev;
    _angleRawprev = angleRawCurr_;
    if (error > +180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

template <class Traits>
void Motor<Traits>::_read() {
    _driverData = _driver.getData();
    _angle.raw += _calcAngleError(_driverData.angle);
    _speed.raw = (double)_driverData.speed;
    _current.raw = _driverData.current;
    _angle.scaled = (static_cast<double>(direction) * _angle.raw / (gearRatio * Traits::innerGearRatio)) - _angleError;
    _speed.scaled = static_cast<double>(direction) * _speed.raw / (gearRatio * Traits::innerGearRatio);
    _current.scaled =  static_cast<double>(direction) * _current.raw;
}

template class Driver<M2006Traits>;
template class Driver<M3508Traits>;
template class Motor<M2006Traits>;
template class Motor<M3508Traits>;
