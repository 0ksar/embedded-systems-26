#include <Arduino.h>

#ifndef Remote_h
#define Remote_h

enum class RemoteMode { Manual, Sonar, Spring };
enum class RemoteAction {
    Locked,
    None,
    Forward,
    Back,
    TurnLeft,
    TurnRight,
    SonarTest,
    SpringTest
};

struct RemoteState {
    RemoteAction action;
    unsigned int value; 
};

typedef void (*RemoteLCDCallback)(const RemoteState& state);
typedef void (*PerformActionCallback)(const RemoteState& state);

class Remote {
    public:
        Remote();
        void configureRemote(
            RemoteLCDCallback updateLCDCallback,
            PerformActionCallback performActionCallback,
            int pin
        );
        void updateAction(uint32_t code);

    private:
        int _pin;
        RemoteState state;
        RemoteLCDCallback updateLCD;
        PerformActionCallback performAction;
        void clearAction();
        void commitAction();
        void unlockAction();   
};

#endif
