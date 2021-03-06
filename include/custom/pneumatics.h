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
    bool wheelIsDown = false;

    //ringles
    bool ringlesToggled = false;
    double ringlesSpeed = 60; //rpm

    void clawGrab(){
        clawLock.set_value(LOW);
        clawToggled = true;
    }

    void clawRelease(){
        clawLock.set_value(HIGH);
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
        tiltLock.set_value(HIGH);
        tiltToggled = true;
    }

    void tiltRelease(){
        tiltLock.set_value(LOW);
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
            //RingleLift.move_velocity(0);
        } else {
            //RingleLift.move_velocity(ringlesSpeed);
        }
        ringlesToggled = !ringlesToggled;
    }

    void setRingles(bool on){
        if(on){
            //RingleLift.move_velocity(ringlesSpeed);
        } else {
            //RingleLift.move_velocity(0);
        }
    }

    void shiftWheelDown(){
        wheelIsDown = true;
        liftShifter.set_value(!wheelIsDown);
    }

    void shiftWheelUp(){
        wheelIsDown = false;
        liftShifter.set_value(!wheelIsDown);
    }

    void shiftWheelToggle(){
        wheelIsDown = !wheelIsDown;
        liftShifter.set_value(!wheelIsDown);
    }

};
#endif // ifndef LIFT
