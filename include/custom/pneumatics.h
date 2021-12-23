#ifndef PNEUMATICS
#define PNEUMATICS
#include "math.h"
class PneumaticsClass
{
private:

public:
    Math myMath;
    bool clawToggled = false;
    bool tiltToggled = false;
    bool backToggled = false;

    //ringles
    bool ringlesToggled = false;
    double ringlesSpeed = 110;

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

    void tiltGrab(){
        tiltLock.set_value(LOW);
        tiltToggled = true;
    }

    void tiltRelease(){
        tiltLock.set_value(HIGH);
        tiltToggled = false;
    }

    void toggletilt(){
        if(tiltToggled){
            tiltRelease();
        } else {
            tiltGrab();
        }
    }

    void backGrab(){
        backLock.set_value(LOW);
        backToggled = true;
    }

    void backRelease(){
        backLock.set_value(HIGH);
        backToggled = false;
    }

    void toggleBack(){
        if(backToggled){
            backRelease();
        } else {
            backGrab();
        }
    }

    //ringles too i guess

    void toggleRingles(){
        if(ringlesToggled){
            Lift.move_velocity(0);
        } else {
            Lift.move_velocity(ringlesSpeed);
        }
        ringlesToggled = !ringlesToggled;
    }

};
#endif // ifndef LIFT
