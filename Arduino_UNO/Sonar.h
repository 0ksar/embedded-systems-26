#include <Arduino.h>

#ifndef Sonar_h
#define Sonar_h

struct SonarState {
    uint8_t angle;
    unsigned int distance;
};

enum class ObstacleSide { None, Left, Center, Right };
struct ObstacleState {
    bool detected;
    ObstacleSide side;
};

typedef void (*SonarLCDCallback)(const SonarState& state);

class Sonar {
    public:
        Sonar();
        ~Sonar();
        void attach(uint8_t pinServo, uint8_t pinTrig, uint8_t pinEcho);
        void configureSonar(SonarLCDCallback updateLCDCallback, uint8_t n);
        SonarState checkDistance(uint8_t angle);
        ObstacleState fastScan(
            uint8_t startAngle,
            uint8_t stopAngle,
            uint8_t stepAngle,
            unsigned int obstacleDistance
        );
        SonarState* fullScan(
            uint8_t startAngle,
            uint8_t stopAngle,
            uint8_t stepAngle
        );

    private:
        uint8_t _pinServo, _pinTrig, _pinEcho;
        SonarLCDCallback updateLCD;
        uint8_t n_cnt;
};

#endif
