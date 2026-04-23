/* 
 * prosta implementacja klasy obsługującej 
 * silniki pojazdu za pośrednictwem modułu L298
 *
 * Sterowanie odbywa się przez:
 * 1)  powiązanie odpowiednich pinów I/O Arduino metodą attach() 
 * 2)  ustalenie prędkości setSpeed*()
 * 3)  wywołanie funkcji ruchu
 *
 * TODO:
 *  - zabezpieczenie przed ruchem bez attach()
 *  - ustawienie domyślnej prędkości != 0
 */

#include <Arduino.h>

#ifndef Wheels_h
#define Wheels_h

// Movement state structure
struct MotionState {
    /*
     * Movement mode:
     * - F: forward
     * - B: backward
     * - S: stop
     * - T: turning, when leftDirection * leftSpeed != rightDirection * rightSpeed
     */ 
    char mode; 
    uint8_t leftSpeed;      // Speed for the left axis.
    uint8_t rightSpeed;     // Speed for the right axis.
    int8_t leftDirection;   // Direction for the left axis.
    int8_t rightDirection;  // Direction for the right axis.
};

// LCD callback type
typedef void (*LCDCallback)(const MotionState& state, int cm);
// Beep callback type
typedef void (*BeepCallback)(uint8_t frequency);

class Wheels {
    public: 
        Wheels();
        /*
         *  pinForward - wejście "naprzód" L298
         *  pinBack    - wejście "wstecz" L298
         *  pinSpeed   - wejście "enable/PWM" L298
         */
        void attachLeft(uint8_t pinForward, uint8_t pinBack, uint8_t pinSpeed);
        void attachRight(uint8_t pinForward, uint8_t pinBack, uint8_t pinSpeed);
        void attach(uint8_t pinRightForward, uint8_t pinRightBack, uint8_t pinRightSpeed,
                    uint8_t pinLeftForward, uint8_t pinLeftBack, uint8_t pinLeftSpeed);
        void configureWheels(
            uint8_t defaultSpeed,
            LCDCallback updateLCDCallback,
            uint16_t LCDfreq,
            BeepCallback updateBeepCallback,
            float wheelDiameter,
            uint8_t slotCount,
            float wheelSpacing
        );

        /*
         *  funkcje ruchu
         */
        void forwardLeft();
        void forwardRight();
        void forward();
        void backLeft();
        void backRight();
        void back();
        void stopLeft();
        void stopRight();
        void stop();
        /*
         *  ustawienie prędkości obrotowej (przez PWM)
         *   - minimalna efektywna wartość 60
         *      może zależeć od stanu naładowania baterii
         */
        void setSpeed(uint8_t);
        void setSpeedRight(uint8_t);
        void setSpeedLeft(uint8_t);

        /*
         *  laboratoria: lista 2
         */
        void goForward(int cm);
        void goBack(int cm);
        void turn(int deg);

        void leftImpulse();
        void rightImpulse();

    private: 
        uint8_t pinsRight[3];
        uint8_t pinsLeft[3];

        // Movement state variables & functions
        MotionState motion;
        void updateMode();
        uint8_t defaultSpeed;

        // Callbacks & related variables
        LCDCallback updateLCD;
        uint16_t frequencyLCD;
        BeepCallback updateBeep;

        // Impulse counting (precise movement speed calc)
        float wheelDiameter;  // (in cm)
        uint8_t slotCount;
        float wheelSpacing;   // (in cm)
        volatile long leftImpulseCount;
        volatile long rightImpulseCount;
};



#endif
