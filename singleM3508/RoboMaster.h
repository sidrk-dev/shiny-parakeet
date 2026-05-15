#ifndef ROBOMASTER_H
#define ROBOMASTER_H

#include <stdint.h>
#include <Arduino.h>
#include "can.h"
#include "CalcPID.h"

inline constexpr uint16_t FEEDBACK_1000HZ = 1000;
inline constexpr uint16_t FEEDBACK_500HZ  = 500;
inline constexpr uint16_t FEEDBACK_250HZ  = 250;
inline constexpr uint16_t FEEDBACK_125HZ  = 125;

inline constexpr uint16_t CAN_1_TO_4_ID = 0x200;
inline constexpr uint16_t CAN_5_TO_8_ID = 0x1FF;

struct M2006Traits {
    static constexpr double currentRangeA = 10.0;
    static constexpr int16_t currentResolution = 10000;
    static constexpr uint16_t encoderResolution = 8192;
    static constexpr double innerGearRatio = 36.0;
};

struct M3508Traits {
    static constexpr double currentRangeA = 20.0;
    static constexpr int16_t currentResolution = 16384;
    static constexpr uint16_t encoderResolution = 8192;
    static constexpr double innerGearRatio = 19.0;
};

enum class MODE { UNUSED, SLEEP, ANGLE, SPEED, CURRENT };
enum class DIRECTION { FWD = 1, REV = -1 };
typedef struct { double angle, speed, current, output; } target_t;

template <class Traits>
class Driver {
public:
    typedef struct { 
        uint8_t id;
        double angle;
        int16_t speed;
        double current;
        uint8_t temperature;
    } driverData_t;
    void setID(uint8_t id_);
    void setCurrent(struct can_frame *sendMsg_, double current_);
    void update(const struct can_frame &readMsg_);
    driverData_t getData();
private:
    typedef struct { uint8_t upper, lower; } bytes_t;
    driverData_t _data = {};
    bytes_t _divide(int16_t val);
    int16_t _connect(bytes_t byte_);
};

template <class Traits>
class Motor {
public:
  MODE mode = MODE::UNUSED;
  DIRECTION direction = DIRECTION::FWD;
  double gearRatio = 1.0;
  struct { pidParam_t angle, speed, current; } pidParam = {};
  target_t target = {};
  void setPidInterval(uint8_t angle_, uint8_t speed_, uint8_t current_);
  void init(uint8_t id_);
  bool refresh(uint32_t micros_, struct can_frame (&sendMsg_)[2], const struct can_frame &readMsg_);
  double getAngle();
  double getSpeed();
  double getCurrent();
  void resetIntegral();
  void resetAngle(double currAngle_);
private:
  uint32_t _refreshCount = 0;
  Driver<Traits> _driver;
  typename Driver<Traits>::driverData_t _driverData;
  struct { CalcPID angle, speed, current; } _calcpid = {};
  struct { uint8_t angle, speed, current; } _interval = { 1, 1, 1 };
  struct { double raw, scaled; } _angle = {}, _speed = {}, _current = {};
  double _angleRawprev = 0.0;
  double _angleError = 0.0;
  double _calcAngleError(double angleRawCurr_);
  void _read();
};

using C610 = Driver<M2006Traits>;
using C620 = Driver<M3508Traits>;
using M2006 = Motor<M2006Traits>;
using M3508 = Motor<M3508Traits>;

extern template class Driver<M2006Traits>;
extern template class Driver<M3508Traits>;
extern template class Motor<M2006Traits>;
extern template class Motor<M3508Traits>;

#endif
