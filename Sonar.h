#include <Arduino.h>

#ifndef Sonar_h
#define Sonar_h

struct SonarState {
    uint8_t angle;
    unsigned int distance;
};

typedef void (*SonarLCDCallback)(const SonarState& state);
typedef void (*SonarObstacleCallback)(const uint8_t turn_angle);

class Sonar {
    public:
        Sonar();
        ~Sonar();
        void attach(uint8_t pinServo, uint8_t pinTrig, uint8_t pinEcho);
        void configureSonar(SonarLCDCallback updateLCDCallback);
        SonarState* scan(uint8_t startAngle, uint8_t stopAngle, uint8_t stepAngle);

    private:
        uint8_t _pinServo, _pinTrig, _pinEcho;
        uint8_t _leftAngle, _rightAngle, _stepAngle;
        SonarState *allDist;
        size_t allDistCount;
        SonarLCDCallback updateLCD;
        SonarObstacleCallback reportObstacle;
};

#endif
