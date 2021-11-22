#ifndef LIFT
#define LIFT
#include "math.h"
class LiftClass
{
private:
    double liftSpeed = 100;
    double clawSpeed = 50; 
public:
    Math myMath;
    //stops motor
    void stopAll()
    {
        Larm.move_velocity(0);
        Rarm.move_velocity(0);
    }

    void stopClaw()
    {
        Claw.move_velocity(0);
    }

    void liftUp(){
        Larm.move_velocity(liftSpeed);
        Rarm.move_velocity(liftSpeed);
    }

    void liftDown(){
        Larm.move_velocity(-liftSpeed);
        Rarm.move_velocity(-liftSpeed);
    }

    void clawUp(){
        Claw.move_velocity(clawSpeed);
    }

    void clawDown(){
        Claw.move_velocity(-clawSpeed);
    }

    void grab(){
        int restult = clawLock.set_value(HIGH);
        printf("%d is returned", restult);
    }

    void letGo(){
        int restult = clawLock.set_value(LOW);
        printf("%d is returned", restult);
    }

};
#endif // ifndef LIFT
