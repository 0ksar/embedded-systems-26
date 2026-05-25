#include "Remote.h"

Remote::Remote()
    : state{RemoteAction::Locked, 0},
      updateLCD(nullptr),
      performAction(nullptr)
{ }

void Remote::configureRemote(
    RemoteLCDCallback updateLCDCallback, 
    PerformActionCallback performActionCallback,
    int pin
) {
    this->updateLCD = updateLCDCallback;
    this->performAction = performActionCallback;
    this->_pin = pin;
}

void Remote::updateAction(uint32_t code) {
    if (code == 0) { clearAction(); return; }  // #
    if (code == 1) { commitAction(); return; }  // OK
    if (this->state.action == RemoteAction::None) {
        switch(code) {
            case 0: this->state.action = RemoteAction::Forward; break;  // UP
            case 1: this->state.action = RemoteAction::TurnRight; break;  // RIGHT
            case 2: this->state.action = RemoteAction::Back; break;  // DOWN
            case 3: this->state.action = RemoteAction::TurnLeft; break;  // LEFT
            case 4: this->state.action = RemoteAction::SonarTest; break;  // 5
            case 5: this->state.action = RemoteAction::SpringTest; break;  // 7
            default: return;
        }
    } else {
        if (this->state.action == RemoteAction::SonarTest || 
            this->state.action == RemoteAction::SpringTest) return;
        int val;
        switch(code) {
            case 1: val = 1; break;
            case 2: val = 2; break;
            case 3: val = 3; break;
            case 4: val = 4; break;
            case 5: val = 5; break;
            case 6: val = 6; break;
            case 7: val = 7; break;
            case 8: val = 8; break;
            case 9: val = 9; break;
            case 0: val = 0; break;
            default: return;
        }
        unsigned int tmp = this->state.value;
        int dig = 0;
        while (tmp > 0) { dig++; tmp /= 10; }
        if (this->state.action == RemoteAction::Locked && dig < 4) {
            this->state.value = this->state.value * 10 + val;
            if (dig == 3) commitAction();
        }
        else if (dig < 3) this->state.value = this->state.value * 10 + val;
    }
    if (this->updateLCD) this->updateLCD(state);
}

void Remote::clearAction() {
    if (this->state.action == RemoteAction::Locked) this->state.value = 0;
    else this->state = {RemoteAction::None, 0};
    if (this->updateLCD) this->updateLCD(this->state);
}

void Remote::commitAction() {
    if (this->state.action == RemoteAction::Locked) unlockAction();
    else this->performAction(this->state);
    clearAction();
}

void Remote::unlockAction() {
    if (this->state.value == this->_pin) {
        this->state.action = RemoteAction::None;
        this->state.value = 0;
    }
    if (this->updateLCD) this->updateLCD(this->state);
}
