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

void Sonar::configureSonar(SonarLCDCallback updateLCDCallback, unsigned int maxDist) {
    this->updateLCD = updateLCDCallback;
    this->maxDistance = maxDist;
}

SonarState Sonar::checkDistance(uint8_t angle) {
    SonarState state;
    unsigned long tot;
    unsigned int dist;
    state.angle = angle;
    servo.write(angle);
    delay(200);
    digitalWrite(this->_pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(this->_pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->_pinTrig, LOW);
    delay(100);
    tot = pulseIn(this->_pinEcho, HIGH, 30000);
    if (tot == 0) {
        dist = this->maxDistance;
    } else {
        dist = tot / 58;
        if (dist > this->maxDistance) dist = this->maxDistance;
    }
    state.distance = dist;
    return state;
}

ObstacleState Sonar::fastScan(
    uint8_t leftAngle, 
    uint8_t centerAngle, 
    uint8_t rightAngle, 
    unsigned int obstacleDist
) {
    SonarState state;
    state = checkDistance(leftAngle);
    if (this->updateLCD) this->updateLCD(state);
    if (state.distance < obstacleDist) return { true, ObstacleSide::Left};
    state = checkDistance(rightAngle);
    if (this->updateLCD) this->updateLCD(state);
    if (state.distance < obstacleDist) return { true, ObstacleSide::Right};
    state = checkDistance(centerAngle);
    if (this->updateLCD) this->updateLCD(state);
    if (state.distance < obstacleDist) return { true, ObstacleSide::Center};
    return { false, ObstacleSide::None};
}

SonarState* Sonar::fullScan(uint8_t startAngle, uint8_t stopAngle, uint8_t stepAngle) {
    SonarState state;
    size_t cnt;
    if (stepAngle == 0 || startAngle > stopAngle) cnt = 0;
    else cnt = ((stopAngle - startAngle) / stepAngle) + 1;
    if (cnt == 0) return nullptr;
    SonarState *res = new SonarState[cnt];
    if (!res) return nullptr;
    size_t idx = 0;
    for (uint8_t angle = startAngle; angle <= stopAngle;angle += stepAngle, ++idx) {
        state = checkDistance(angle);
        if (this->updateLCD) this->updateLCD(state);
        res[idx] = state;
    }
    return res;
}
