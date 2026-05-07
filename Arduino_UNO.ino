#include "Wheels.h"
#include <LiquidCrystal_I2C.h>
#include "TimerOne.h"

byte LCDAddress = 0x27;
LiquidCrystal_I2C lcd(LCDAddress, 16, 2);

#define BEEPER 13

Wheels w;
volatile char cmd;

// -- Config variables --
uint8_t DEF_SPEED = 150;                   // Speed value used in goForward and goBack
uint16_t LCD_FREQ = 500;                   // LCD refresh frequency (in ms)
bool ANIMATE = true;                       // LCD animation flag (false = off)
const unsigned long BEEP_PERIOD = 500000;  // Less = faster default beeping
// BEEP_PERIOD = 500000 -> 75 speed: 1Hz, 150: 2Hz, 225: 3Hz, ...
float WHEEL_DIAM = 6;                      // Wheel diameter (in cm)
uint8_t SLOT_CNT = 12;                     // Number of slots on the wheel
float WHEEL_SPAC = 15;                     // Wheel spacing (in cm)

// -- NOT config variables (do not touch) --
unsigned long animationStep = 0;           // Last animation state change (time, ms)
uint8_t beepFreqPrev = -1;                 // Previous beeping frequency value (0-4)
volatile uint8_t prevA0 = 0;               // Previous A0 pin state
volatile uint8_t prevA1 = 0;               // Previous A1 pin state

// -- ARDUINO DEVICES functions --

// Returns the length of any given integer (written as a string, ex. 100 -> 3).
int getLength(int n) {
  if (n == 0) return 1;
  int l = 0;
  while (n > 0) { n /= 10; l++; }
  return l;
}

/*
 * Updates LCD display.
 *
 * LCD size is 2x16 (2 rows, 16 characters each).
 * First row is used to display remaining cm while using goForward() or goBack() functions 
 * from the Wheels class. Second, for showing current speeds for each axis (left, right) 
 * and in the middle, movement state: FRWD (forward), BACK, STOP, TURN, unknown state: ????
 * with simple blinking animation (can be turned off with the ANIMATE variable). 
 * Also, the LCD refresh frequency can be adjusted using the LCD_FREQ variable (set in ms).
 */
void updateLCD(const MotionState& state, int cm) {
  // Clear first line
  lcd.setCursor(0,0);
  lcd.print("                ");
  // First line: remaining distance
  if (cm > 0) {
    lcd.setCursor((13-getLength(cm))/2,0);
    lcd.print(cm);
    lcd.print(" cm");
  }
  // Clear second line 
  lcd.setCursor(0,1);
  lcd.print("                ");
  // Second line left: left engine status
  lcd.setCursor(0,1);
  lcd.print(state.leftDirection < 0 ? "-" : "");
  lcd.print(state.leftSpeed);
  // Second line middle: movement state (animated)
  const char* animationText;
  switch(state.mode)
  {
    case 'F': animationText = "FRWD"; break;
    case 'B': animationText = "BACK"; break;
    case 'S': animationText = "STOP"; break;
    case 'T': animationText = "TURN"; break;
    default: animationText = "????";
  }
  // Blinking animation
  lcd.setCursor(6,1);
  unsigned long now = millis();
  if (ANIMATE && now - animationStep > 250) {
    lcd.print("    ");
    animationStep = now;
  } else {
    lcd.print(animationText);
  }
  // Second line right: right engine status
  int rightLength = (state.rightDirection < 0 ? 1 : 0) + getLength(state.rightSpeed);
  lcd.setCursor(16-rightLength,1);
  lcd.print(state.rightDirection < 0 ? "-" : "");
  lcd.print(state.rightSpeed);
}

/*
 * Changes beeping frequency. 
 *
 * For BEEP_PERIOD = 500000:
 *   - frequency 0: no beeping,
 *   - min speed (60): 0.8Hz,
 *   - speed 75: 1Hz,
 *   - speed 150: 2Hz,
 *   - speed 225: 3Hz,
 *   - max speed (255): 3.4Hz.
 * Catches pointless changes, when new frequency is the same as the previous one.
 */
void updateBeep(uint8_t frequency) {
  if (beepFreqPrev == frequency) return;  // If nothing changed - do nothing.
  beepFreqPrev = frequency;
  Timer1.detachInterrupt();
  if (frequency == 0) {  // Stop beeping if frequency = 0.
    digitalWrite(BEEPER, LOW);
    return;
  }
  Timer1.attachInterrupt(doBeep, BEEP_PERIOD / frequency);
}

// Beeps.
void doBeep() {
  digitalWrite(BEEPER, digitalRead(BEEPER) ^ 1);
}

ISR(PCINT1_vect) {
  uint8_t pins = PINC;
  uint8_t a0 = (pins & (1 << PC0)) != 0;
  uint8_t a1 = (pins & (1 << PC1)) != 0;

  // Count only HIGH -> LOW
  if (!a0 && prevA0) w.leftImpulse();
  if (!a1 && prevA1) w.rightImpulse();
  prevA0 = a0;
  prevA1 = a1;
}

// -- ARDUINO functions --

void setup() {
  w.attach(2,3,5,6,7,11);

  pinMode(BEEPER, OUTPUT);
  Timer1.initialize();
  updateBeep(0);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  prevA0 = (PINC & (1 << PC0)) != 0;
  prevA1 = (PINC & (1 << PC1)) != 0;
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);
  
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println("goForward(cm): F<cm>");
  Serial.println("goBack(cm): B<cm>");
  Serial.println("test: T<cm> (forwards, spin, backwards)");
  Serial.println("Manual: WAD, ZXC, S");

  lcd.init();
  lcd.backlight();

  w.configureWheels(
    DEF_SPEED, 
    updateLCD, 
    LCD_FREQ, 
    updateBeep, 
    WHEEL_DIAM, 
    SLOT_CNT, 
    WHEEL_SPAC
  );
}

void loop() {
  w.goForward(100);
  delay(3000);
  w.goBack(100);
  delay(3000);

  // while(Serial.available())
  // {
  //   cmd = Serial.read();
  //   switch(cmd)
  //   {
  //     case 'W': w.forward(); break;
  //     case 'X': w.back(); break;
  //     case 'A': w.forwardLeft(); break;
  //     case 'D': w.forwardRight(); break;
  //     case 'Z': w.backLeft(); break;
  //     case 'C': w.backRight(); break;
  //     case 'S': w.stop(); break;
  //     case '1': w.setSpeedLeft(75); break;
  //     case '2': w.setSpeedLeft(150); break;
  //     case '3': w.setSpeedLeft(225); break;
  //     case '8': w.setSpeedRight(75); break;
  //     case '9': w.setSpeedRight(150); break;
  //     case '0': w.setSpeedRight(225); break;
  //     case '5': w.setSpeed(150); break;
  //     case 'F': { int cm = Serial.parseInt(); w.goForward(cm); break; }
  //     case 'B': { int cm = Serial.parseInt(); w.goBack(cm); break; }
  //     case 'T': { int cm = Serial.parseInt(); w.goForward(cm); w.turn(360); w.goBack(cm); break; }
  //   }
  // }
}
