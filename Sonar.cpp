#include <Servo.h>
#include "Sonar.h"

Servo servo;

Sonar::Sonar()
    : updateLCD(nullptr),
      allDist(nullptr),
      allDistCount(0)
{ }

Sonar::~Sonar() {
    delete[] allDist;
    allDist = nullptr;
    allDistCount = 0;
}

void Sonar::attach(uint8_t pinServo, uint8_t pinTrig, uint8_t pinEcho) {
    this->_pinServo = pinServo;
    this->_pinTrig = pinTrig;
    this->_pinEcho = pinEcho;

    pinMode(this->_pinTrig, OUTPUT);
    digitalWrite(this->_pinTrig, LOW);
    pinMode(this->_pinEcho, INPUT);
    servo.attach(this->_pinServo);
    servo.write(90);
    delay(100);
}

void Sonar::configureSonar(SonarLCDCallback updateLCDCallback) {
    this->updateLCD = updateLCDCallback;
}

SonarState* Sonar::scan(uint8_t startAngle, uint8_t stopAngle, uint8_t stepAngle) {
    unsigned long tot;
    unsigned int dist;
    SonarState state;
    size_t cnt;
    if (stepAngle == 0 || startAngle > stopAngle) cnt = 0;
    else cnt = ((stopAngle - startAngle) / stepAngle) + 1;
    if (cnt == 0) return nullptr;
    SonarState *res = new SonarState[cnt];
    if (!res) return nullptr;
    size_t idx = 0;
    for (
        uint8_t angle = startAngle; 
        angle <= stopAngle;
        angle += stepAngle, ++idx
    ) {
        state.angle = angle;
        servo.write(angle);
        digitalWrite(this->_pinTrig, HIGH);
        delay(10);
        digitalWrite(this->_pinTrig, LOW);
        tot = pulseIn(this->_pinEcho, HIGH);
        dist = tot / 58;
        state.distance = dist;
        if (this->updateLCD) this->updateLCD(state);
        res[idx] = state;
    }
    return res;
}
