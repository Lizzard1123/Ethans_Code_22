#ifndef PNEUMATICS
#define PNEUMATICS
#include "math.h"
class PneumaticsClass
{
private:

public:
    Math myMath;
    bool clawToggled = false;
    bool frontToggled = false;
    bool backToggled = false;

    void clawGrab(){
        clawLock.set_value(HIGH);
        clawToggled = true;
    }

    void clawRelease(){
        clawLock.set_value(LOW);
        clawToggled = false;
    }

    void toggleClaw(){
        if(clawToggled){
            clawRelease();
        } else {
            clawGrab();
        }
    }

    void frontGrab(){
        frontLock.set_value(HIGH);
        frontToggled = true;
    }

    void frontRelease(){
        frontLock.set_value(LOW);
        frontToggled = false;
    }

    void toggleFront(){
        if(frontToggled){
            frontRelease();
        } else {
            frontGrab();
        }
    }

    void backGrab(){
        backLock.set_value(HIGH);
        backToggled = true;
    }

    void backRelease(){
        backLock.set_value(LOW);
        backToggled = false;
    }

    void toggleBack(){
        if(backToggled){
            backRelease();
        } else {
            backGrab();
        }
    }

};
#endif // ifndef LIFT
