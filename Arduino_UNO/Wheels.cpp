#include <Arduino.h>
#include "Wheels.h"

#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

Wheels::Wheels() 
    : motion{'S', 0, 0, 0, 0},
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

void Wheels::attachLeft(uint8_t pF, uint8_t pB, uint8_t pS) {
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}

void Wheels::attachRight(uint8_t pF, uint8_t pB, uint8_t pS) {
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}

void Wheels::attach(uint8_t pRF, uint8_t pRB, uint8_t pRS, uint8_t pLF, uint8_t pLB, uint8_t pLS) {
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}

void Wheels::configureWheels(
    uint8_t defaultSpeed,
    MotionLCDCallback updateLCDCallback,
    uint16_t freqLCD,
    BeepCallback updateBeepCallback,
    float wheelDiameter,
    uint8_t slotCount,
    float wheelSpacing
) {
    this->defaultSpeed = defaultSpeed;
    this->updateLCD = updateLCDCallback;
    this->frequencyLCD = freqLCD;
    this->updateBeep = updateBeepCallback;
    this->wheelDiameter = wheelDiameter;
    this->slotCount = slotCount;
    this->wheelSpacing = wheelSpacing;
}

void Wheels::forwardLeft() {
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
    motion.leftDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::forwardRight() {
    SET_MOVEMENT(pinsRight, HIGH, LOW);
    motion.rightDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::forward() {
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
    SET_MOVEMENT(pinsRight, HIGH, LOW);
    motion.leftDirection = 1;
    motion.rightDirection = 1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::backLeft() {
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
    motion.leftDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::backRight() {
    SET_MOVEMENT(pinsRight, LOW, HIGH);
    motion.rightDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::back() {
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
    SET_MOVEMENT(pinsRight, LOW, HIGH);
    motion.leftDirection = -1;
    motion.rightDirection = -1;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::stopLeft() {
    SET_MOVEMENT(pinsLeft, LOW, LOW);
    motion.leftDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::stopRight() {
    SET_MOVEMENT(pinsRight, LOW, LOW);
    motion.rightDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::stop() {
    SET_MOVEMENT(pinsLeft, LOW, LOW);
    SET_MOVEMENT(pinsRight, LOW, LOW);
    motion.leftDirection = 0;
    motion.rightDirection = 0;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
}

void Wheels::setSpeedLeft(uint8_t s) {
    motion.leftSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsLeft[2], s);
}

void Wheels::setSpeedRight(uint8_t s) {
    motion.rightSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsRight[2], s);
}

void Wheels::setSpeed(uint8_t s) {
    motion.leftSpeed = s;
    motion.rightSpeed = s;
    updateMode();
    if (this->updateLCD != nullptr) this->updateLCD(motion, -1);
    analogWrite(this->pinsLeft[2], s);
    analogWrite(this->pinsRight[2], s);
}

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
    if (deg > 0) {
        this->forwardLeft();
        this->backRight();
    } else {
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

void Wheels::leftImpulse() {
    this->leftImpulseCount++;
}

void Wheels::rightImpulse() {
    this->rightImpulseCount++;
}

void Wheels::updateMode() {
    int vL = motion.leftDirection * motion.leftSpeed;
    int vR = motion.rightDirection * motion.rightSpeed;
    if (vL == 0 && vR == 0) {
        motion.mode = 'S';
    } else if (vL != vR) {
        motion.mode = 'T';
    } else if (vL == vR && vL > 0) {
        motion.mode = 'F';
    } else {
        motion.mode = 'B';
    }
    if (updateBeep != nullptr) {
        if (vL + vR < 0) {
            uint8_t beepFreq = (((motion.leftSpeed + motion.rightSpeed) / 2) + 74) / 75;
            if (beepFreq > 4) beepFreq = 4;
            this->updateBeep(beepFreq);
        } else {
            this->updateBeep(0);
        }
    }
}
