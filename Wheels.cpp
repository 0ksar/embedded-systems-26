#include <Arduino.h>
#include "Wheels.h"

#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

Wheels::Wheels() 
    : motion{'S', 0, 0, 0, 0}, // {stop, lspeed 0, rspeed 0, ldir 0 (stop), rdir 0}
      defaultSpeed(0),
      updateLCD(nullptr),
      frequencyLCD(0),
      updateBeep(nullptr),
      wheelDiameter(0),
      slotCount(0),
      wheelSpacing(0),
      leftImpulseCount(0),
      rightImpulseCount(0)
{ }

// -- CONFIG functions --

// Attaches pin configuration for the left axis (implemented initially).
void Wheels::attachLeft(uint8_t pF, uint8_t pB, uint8_t pS) {
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}

// Attaches pin configuration for the right axis (implemented initially).
void Wheels::attachRight(uint8_t pF, uint8_t pB, uint8_t pS) {
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}

// Attaches pin configuration for both axles (implemented initially).
void Wheels::attach(uint8_t pRF, uint8_t pRB, uint8_t pRS, uint8_t pLF, uint8_t pLB, uint8_t pLS) {
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}

// Configures all required variables and callbacks.
void Wheels::configureWheels(
    uint8_t defaultSpeed,               // Default speed value used in goForward, goBack, turn
    LCDCallback updateLCDCallback,      // Function for updating the LCD (remaining cm display)
    uint16_t freqLCD,                   // LCD update frequency constant (for goForward() & ...)
    BeepCallback updateBeepCallback,    // Function for updating the beep frequency
    float wheelDiameter,                // Wheel diameter (in cm)
    uint8_t slotCount,                  // Number of slots on the wheel
    float wheelSpacing                  // Gap between the wheels
) {
    this->defaultSpeed = defaultSpeed;
    this->updateLCD = updateLCDCallback;
    this->frequencyLCD = freqLCD;
    this->updateBeep = updateBeepCallback;
    this->wheelDiameter = wheelDiameter;
    this->slotCount = slotCount;
    this->wheelSpacing = wheelSpacing;
}

// -- FORWARD functions --

// Sets movement direction to forward for the left axis.
void Wheels::forwardLeft() {
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
    motion.leftDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Sets movement direction to forward for the right axis.
void Wheels::forwardRight() {
    SET_MOVEMENT(pinsRight, HIGH, LOW);
    motion.rightDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Sets movement direction to forward for both axles.
void Wheels::forward() {
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
    SET_MOVEMENT(pinsRight, HIGH, LOW);
    motion.leftDirection = 1;
    motion.rightDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// -- BACKWARD functions -- 

// Sets movement direction to backward for the left axis.
void Wheels::backLeft() {
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
    motion.leftDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Sets movement direction to backward for the right axis.
void Wheels::backRight() {
    SET_MOVEMENT(pinsRight, LOW, HIGH);
    motion.rightDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Sets movement direction to backward for both axles.
void Wheels::back() {
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
    SET_MOVEMENT(pinsRight, LOW, HIGH);
    motion.leftDirection = -1;
    motion.rightDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// -- STOP functions --

// Stops left axis.
void Wheels::stopLeft() {
    SET_MOVEMENT(pinsLeft, LOW, LOW);
    motion.leftDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Stops right axis.
void Wheels::stopRight() {
    SET_MOVEMENT(pinsRight, LOW, LOW);
    motion.rightDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// Stops both axles.
void Wheels::stop() {
    SET_MOVEMENT(pinsLeft, LOW, LOW);
    SET_MOVEMENT(pinsRight, LOW, LOW);
    motion.leftDirection = 0;
    motion.rightDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

// -- SPEED functions --

// Sets speed for the left axis.
void Wheels::setSpeedLeft(uint8_t s) {
    motion.leftSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsLeft[2], s);
}

// Sets speed for the right axis.
void Wheels::setSpeedRight(uint8_t s) {
    motion.rightSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsRight[2], s);
}

// Sets speed for both axles.
void Wheels::setSpeed(uint8_t s) {
    motion.leftSpeed = s;
    motion.rightSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsLeft[2], s);
    analogWrite(this->pinsRight[2], s);
}

/* 
 * -- LAB 2 and future --
 *
 * goForward and goBack functions are used to move the car 
 * in the certain direction by a given distance (in cm).
 * 
 * Initially, they made use of the delay() function, but this
 * had to be changed because of the constant updating of the LCD data
 * required for LAB 3. Now they use callback functions implemented
 * in the lab2_wheelsBasic.ino file, which are attached in attachCallbacks().
 * These callbacks include upadating the LCD and changing the beep interval.
 * 
 * turn() function is used to rotate the car by given degree. 
 * 
 * Handles both ways of rotation, left and right. For the rotation itself,
 * the previous speed settings are saved, and temporarilly are set different,
 * constant speed values for each axis. After completing the movement action, 
 * initial settings are restored.
 */

/*
 * Moves the car forward by given distance.
 * LAB 2: uses delay(cm * ms_per_cm) to determine how long will it take for the
 * car to move given distance.
 * LAB 3: uses the same idea, but instead of delay(), uses millis() function to keep
 * track of the time passed from the start of the movement, while constantly sending data
 * for the LCD to display (remaining cm) using right callback function.
 * LAB 4: makes use of the impulse counting from the slot sensors installed on each wheel 
 * axles (left & right), to calculate the speed accurately and based on that, determine 
 * traveled distance.
 */
void Wheels::goForward(int cm) {
    if (
        cm <= 0 || 
        this->defaultSpeed == 0 || 
        this->updateLCD == nullptr || 
        this->frequencyLCD == 0 || 
        this->wheelDiameter == 0.0 || 
        this->slotCount == 0
    ) return;

    const float impulseRatio = (PI * this->wheelDiameter) / this->slotCount;
    const long impusleTarget = (cm / impulseRatio) + 0.5;

    noInterrupts();
    this->leftImpulseCount = 0;
    this->rightImpulseCount = 0;
    interrupts();

    this->setSpeed(this->defaultSpeed);
    this->forward();
    unsigned long stepLCD = 0;
    while (true) {
        long left, right;
        noInterrupts();
        left = this->leftImpulseCount;
        right = this->rightImpulseCount;
        interrupts();
        long minImpulseCount = (left < right) ? left : right;
        if (minImpulseCount >= impusleTarget) break;
        unsigned long now = millis();
        if (now - stepLCD >= this->frequencyLCD) {
            float distance = minImpulseCount * impulseRatio;
            int remaining = (int)(cm - distance + 0.999f);
            if (remaining < 0) remaining = 0;
            this->updateLCD(this->motion, remaining);
            stepLCD = now;
        }
    }
    this->stop();
    this->updateLCD(this->motion, 0);
}

/*
 * Moves the car backward by given distance.
 * LAB 2, 3, 4: similar to goForward function.
 */
void Wheels::goBack(int cm) {
    if (
        cm <= 0 || 
        this->defaultSpeed == 0 || 
        this->updateLCD == nullptr || 
        this->frequencyLCD == 0 || 
        this->wheelDiameter == 0.0 || 
        this->slotCount == 0
    ) return;

    const float impulseRatio = (PI * this->wheelDiameter) / (float)this->slotCount;
    const long impusleTarget = (cm / impulseRatio) + 0.5;

    noInterrupts();
    this->leftImpulseCount = 0;
    this->rightImpulseCount = 0;
    interrupts();

    this->setSpeed(this->defaultSpeed);
    this->back();
    unsigned long stepLCD = 0;
    while (true) {
        long left, right;
        noInterrupts();
        left = this->leftImpulseCount;
        right = this->rightImpulseCount;
        interrupts();
        long minImpulseCount = (left < right) ? left : right;
        if (minImpulseCount >= impusleTarget) break;

        unsigned long now = millis();
        if (now - stepLCD >= this->frequencyLCD) {
            float distance = minImpulseCount * impulseRatio;
            int remaining = (int)(cm - distance + 0.999f);
            if (remaining < 0) remaining = 0;
            this->updateLCD(this->motion, remaining);
            stepLCD = now;
        }
    }
    this->stop();
}

// Turns the car by a given degree.
void Wheels::turn(int deg) {
    if (
        deg == 0 || 
        this->defaultSpeed == 0 || 
        this->slotCount == 0 || 
        this->wheelDiameter == 0.0 || 
        this->wheelSpacing == 0.0
    ) return;

    const float impulseRatio = (PI * this->wheelDiameter) / (float)this->slotCount;
    const float turnDistance = PI * this->wheelSpacing * (abs(deg) / 360.0);
    const long impulseTarget = (long)(turnDistance / impulseRatio + 0.5);

    noInterrupts();
    this->leftImpulseCount = 0;
    this->rightImpulseCount = 0;
    interrupts();

    this->setSpeed(this->defaultSpeed);
    if (deg > 0) {  // Turn right
        this->forwardLeft();
        this->backRight();
    } else {  // Turn left
        this->backLeft();
        this->forwardRight();
    }
    while(true) {
        long left, right;
        noInterrupts();
        left = this->leftImpulseCount;
        right = this->rightImpulseCount;
        interrupts();
        long minImpulseCount = (left < right) ? left : right;
        if (minImpulseCount >= impulseTarget) break;
    }
    this->stop();
}

// -- IMPULSE COUNTERS functions --

void Wheels::leftImpulse() {
    this->leftImpulseCount++;
}

void Wheels::rightImpulse() {
    this->rightImpulseCount++;
}

// -- MOTION STATE functions --

/*
 * After every movement settings change, sets new movement mode (if required).
 * Initializes beeping action, when the car is moving backwards (backward turning included),
 * calculates frequency based on the average speed.
 */
void Wheels::updateMode() {
    int vL = motion.leftDirection * motion.leftSpeed;
    int vR = motion.rightDirection * motion.rightSpeed;

    if (vL == 0 && vR == 0) {
        motion.mode = 'S';  // Stop
    } else if (vL != vR) {
        motion.mode = 'T';  // Turning
    } else if (vL == vR && vL > 0) {
        motion.mode = 'F';  // Forward
    } else {
        motion.mode = 'B';  // Backward
    }

    if (updateBeep != nullptr) {
        if (vL + vR < 0) {  // If moving backwards, start beeping (backward turning included).
            uint8_t beepFreq = (((motion.leftSpeed + motion.rightSpeed) / 2) + 74) / 75;
            if (beepFreq > 4) beepFreq = 4;
            this->updateBeep(beepFreq);
        } else {  // Stop beeping otherwise.
            this->updateBeep(0);
        }
    }
}
