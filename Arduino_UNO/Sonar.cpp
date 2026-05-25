#include <Servo.h>
#include "Sonar.h"

Servo servo;

Sonar::Sonar()
    : updateLCD(nullptr),
      n_cnt(1)
{ }

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

void Sonar::configureSonar(SonarLCDCallback updateLCDCallback, uint8_t n) {
    this->updateLCD = updateLCDCallback;
    this->n_cnt = n;
}

SonarState Sonar::checkDistance(uint8_t angle) {
    SonarState state;
    unsigned long tot;
    unsigned int dist;
    unsigned int dists[this->n_cnt];
    state.angle = angle;
    servo.write(angle);
    delay(200);
    for (uint8_t i = 0; i < this->n_cnt; ++i) {
        digitalWrite(this->_pinTrig, LOW);
        delayMicroseconds(2);
        digitalWrite(this->_pinTrig, HIGH);
        delayMicroseconds(10);
        digitalWrite(this->_pinTrig, LOW);
        tot = pulseIn(this->_pinEcho, HIGH, 30000);
        dists[i] = tot / 58;
    }
    if (dists[0] > dists[1]) {
        unsigned int t = dists[0];
        dists[0] = dists[1];
        dists[1] = t;
    }
    if (dists[1] > dists[2]) {
        unsigned int t = dists[1];
        dists[1] = dists[2];
        dists[2] = t;
    }
    if (dists[0] > dists[1]) {
        unsigned int t = dists[0];
        dists[0] = dists[1];
        dists[1] = t;
    }
    dist = dists[1];
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
