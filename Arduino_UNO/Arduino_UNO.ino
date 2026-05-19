#include "Wheels.h"
#include "Sonar.h"
#include <LiquidCrystal_I2C.h>
#include "TimerOne.h"

byte LCDAddress = 0x27;
LiquidCrystal_I2C lcd(LCDAddress, 16, 2);

#define BEEPER 13

Wheels w;
Sonar s;

const uint8_t CRUISE_SPEED = 150;
const uint8_t DEF_SPEED = 250;

const uint16_t LCD_FREQ = 500;
const bool ANIMATE = false;
const unsigned long BEEP_PERIOD = 500000;
const bool USE_BEEPER_TIMER = false;

const float WHEEL_DIAM = 6.46;
const uint8_t SLOT_CNT = 18;
const float WHEEL_SPAC = 24.37;

// Range 0 - 180
const uint8_t SONAR_FAST_LEFT = 45;
const uint8_t SONAR_FAST_CENTER = 90;
const uint8_t SONAR_FAST_RIGHT = 135;

const uint16_t SONAR_FREQ = 200;
const unsigned int OBSTACLE_MIN_DIST = 60;

unsigned long animationStep = 0;
uint8_t beepFreqPrev = -1;
volatile uint8_t prevA0 = 0;
volatile uint8_t prevA1 = 0;
volatile bool turning = false;
unsigned long scanStep = 0;

int getLength(int n) {
  if (n == 0) return 1;
  int l = 0;
  while (n > 0) { n /= 10; l++; }
  return l;
}

void updateMotionLCD(const MotionState& state, int cm) {
  if (cm > 0) {
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor((13-getLength(cm))/2,0);
    lcd.print(cm);
    lcd.print(" cm");
  }
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print(state.leftDirection < 0 ? "-" : "");
  lcd.print(state.leftSpeed);
  const char* animationText;
  switch(state.mode)
  {
    case 'F': animationText = "FRWD"; break;
    case 'B': animationText = "BACK"; break;
    case 'S': animationText = "STOP"; break;
    case 'T': animationText = "TURN"; break;
    default: animationText = "????";
  }
  lcd.setCursor(6,1);
  unsigned long now = millis();
  if (ANIMATE && now - animationStep > 250) {
    lcd.print("    ");
    animationStep = now;
  } else {
    lcd.print(animationText);
  }
  int rightLength = (state.rightDirection < 0 ? 1 : 0) + getLength(state.rightSpeed);
  lcd.setCursor(16-rightLength,1);
  lcd.print(state.rightDirection < 0 ? "-" : "");
  lcd.print(state.rightSpeed);
}

void updateSonarLCD(const SonarState& state) {
  lcd.setCursor(0,0);
  lcd.print("                ");
  const int8_t fixedAngle = state.angle - 90;
  lcd.setCursor(0,0);
  lcd.print(fixedAngle);
  lcd.print("*");
  lcd.setCursor((11-getLength(state.distance)),0);
  lcd.print("dst: ");
  lcd.print(state.distance);
}

void updateBeep(uint8_t frequency) {
  if (beepFreqPrev == frequency || !USE_BEEPER_TIMER) return;
  beepFreqPrev = frequency;
  Timer1.detachInterrupt();
  if (frequency == 0) {
    digitalWrite(BEEPER, LOW);
    return;
  }
  Timer1.attachInterrupt(doBeep, BEEP_PERIOD / frequency);
}

void doBeep() {
  digitalWrite(BEEPER, digitalRead(BEEPER) ^ 1);
}

ISR(PCINT1_vect) {
  uint8_t pins = PINC;
  uint8_t a0 = (pins & (1 << PC0)) != 0;
  uint8_t a1 = (pins & (1 << PC1)) != 0;
  if (!a0 && prevA0) w.leftImpulse();
  if (!a1 && prevA1) w.rightImpulse();
  prevA0 = a0;
  prevA1 = a1;
}

unsigned int distanceMedian(uint8_t angle) {
    const uint8_t N = 3;
    unsigned int distances[N];
    for (uint8_t i = 0; i < N; ++i) { distances[i] = s.checkDistance(angle).distance; }
    if (distances[0] > distances[1]) {
        unsigned int t = distances[0];
        distances[0] = distances[1];
        distances[1] = t;
    }
    if (distances[1] > distances[2]) {
        unsigned int t = distances[1];
        distances[1] = distances[2];
        distances[2] = t;
    }
    if (distances[0] > distances[1]) {
        unsigned int t = distances[0];
        distances[0] = distances[1];
        distances[1] = t;
    }
    return distances[1];
}

void sonarFastScan(uint8_t leftAngle, uint8_t centerAngle, uint8_t rightAngle) {
  ObstacleState res = s.fastScan(leftAngle, centerAngle, rightAngle, OBSTACLE_MIN_DIST);
  if (!res.detected) return;
  w.stop();
  unsigned int distance;
  switch (res.side) {
    case ObstacleSide::Center:
      distance = distanceMedian(leftAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn(leftAngle - 90);
        delay(100);
        break;
      }
      distance = distanceMedian(rightAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn(rightAngle - 90);
        delay(100);
        break;
      }
      turning = true;
      w.turn(180);
      delay(100);
      turning = false;
      sonarFullScan(0, 180, 20);
      break;
    case ObstacleSide::Left:
      distance = distanceMedian(rightAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn(rightAngle - 90);
        delay(100);
        break;
      }
      distance = distanceMedian(centerAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn((rightAngle - 90) / 2);
        delay(100);
        break;
      }
      turning = true;
      w.turn(180);
      delay(100);
      turning = false;
      sonarFullScan(0, 180, 20);
      break;
    case ObstacleSide::Right:
      distance = distanceMedian(leftAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn(leftAngle - 90);
        delay(100);
        break;
      }
      distance = distanceMedian(centerAngle);
      if (distance >= OBSTACLE_MIN_DIST) {
        w.turn((leftAngle - 90) / 2);
        delay(100);
        break;
      }
      turning = true;
      w.turn(-180);
      delay(100);
      turning = false;
      sonarFullScan(0, 180, 20);
      break;
    default:
      break;
  }
  w.setSpeed(CRUISE_SPEED);
  w.forward();
}

void sonarFullScan(uint8_t startAngle, uint8_t stopAngle, uint8_t stepAngle) {
  if (turning) return;
  SonarState *distances = s.fullScan(startAngle, stopAngle, stepAngle);
  uint8_t count = (stepAngle == 0 || stopAngle < startAngle)
    ? 0
    : ((stopAngle - startAngle) / stepAngle) + 1;
  if (count == 0 || distances == nullptr) {
    delete[] distances;
    return;
  }
  uint8_t bestAngle = distances[0].angle;
  unsigned int bestDistance = distances[0].distance;
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t angle = distances[i].angle;
    unsigned int distance = distances[i].distance;
    if (distance > bestDistance) {
      bestAngle = angle;
      bestDistance = distance;
    }
  }
  delete[] distances;
  if (bestDistance > OBSTACLE_MIN_DIST) {
    w.turn(bestAngle - 90);
    delay(100);
  } else {
    turning = true;
    w.turn(bestAngle - 90 >= 0 ? 90 : -90);
    delay(100);
    turning = false;
    sonarFullScan(0, 180, 20);
  }
}

void setup() {
  w.attach(2,4,5,6,7,11);
  s.attach(3, A3, A2);

  pinMode(BEEPER, OUTPUT);
  if (USE_BEEPER_TIMER) Timer1.initialize();
  updateBeep(0);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  prevA0 = (PINC & (1 << PC0)) != 0;
  prevA1 = (PINC & (1 << PC1)) != 0;
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);
  
  Serial.begin(9600);
  Serial.setTimeout(50);

  lcd.init();
  lcd.backlight();

  w.configureWheels(
    DEF_SPEED, 
    updateMotionLCD, 
    LCD_FREQ, 
    updateBeep, 
    WHEEL_DIAM, 
    SLOT_CNT, 
    WHEEL_SPAC
  );

  s.configureSonar(
    updateSonarLCD
  );

  w.setSpeed(CRUISE_SPEED);
  w.forward();
  scanStep = millis();
}

void loop() {
  if (!turning && (millis() - scanStep >= SONAR_FREQ)) {
    sonarFastScan(SONAR_FAST_LEFT, SONAR_FAST_CENTER, SONAR_FAST_RIGHT);
    scanStep = millis();
  }
}
