#include <Arduino.h>

#ifndef Wheels_h
#define Wheels_h

struct MotionState {
    char mode; 
    uint8_t leftSpeed;
    uint8_t rightSpeed;
    int8_t leftDirection;
    int8_t rightDirection;
};

typedef void (*MotionLCDCallback)(const MotionState& state, int cm);
typedef void (*BeepCallback)(uint8_t frequency);

class Wheels {
    public: 
        Wheels();
        void attachLeft(uint8_t pinForward, uint8_t pinBack, uint8_t pinSpeed);
        void attachRight(uint8_t pinForward, uint8_t pinBack, uint8_t pinSpeed);
        void attach(uint8_t pinRightForward, uint8_t pinRightBack, uint8_t pinRightSpeed,
                    uint8_t pinLeftForward, uint8_t pinLeftBack, uint8_t pinLeftSpeed);
        void configureWheels(
            uint8_t defaultSpeed,
            MotionLCDCallback updateLCDCallback,
            uint16_t LCDfreq,
            BeepCallback updateBeepCallback,
            float wheelDiameter,
            uint8_t slotCount,
            float wheelSpacing
        );
        void forwardLeft();
        void forwardRight();
        void forward();
        void backLeft();
        void backRight();
        void back();
        void stopLeft();
        void stopRight();
        void stop();
        void setSpeed(uint8_t);
        void setSpeedRight(uint8_t);
        void setSpeedLeft(uint8_t);
        void goForward(int cm);
        void goBack(int cm);
        void turn(int deg);
        void leftImpulse();
        void rightImpulse();

    private: 
        uint8_t pinsRight[3];
        uint8_t pinsLeft[3];
        MotionState motion;
        void updateMode();
        uint8_t defaultSpeed;
        MotionLCDCallback updateLCD;
        uint16_t frequencyLCD;
        BeepCallback updateBeep;
        float wheelDiameter;
        uint8_t slotCount;
        float wheelSpacing;
        volatile long leftImpulseCount;
        volatile long rightImpulseCount;
};

#endif
