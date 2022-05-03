#ifndef MOVE
#define MOVE
//include subsytem classes here

#include "math.h"
class RobotMovement
{
private:
    //values to store altered movement axis before computation
    double LXaxis = 0;
    double LYaxis = 0;
    double RXaxis = 0;
    double RYaxis = 0;

    //higher the number the slower the turn
    double rotationTune = 1.5;

    //deadzone : min num to be detected from joystick
    double deadZone = 5;

public:
// takes in inputs and makes final speed for all motors
    double FLspeed;
    double FRspeed;
    double BLspeed;
    double BRspeed;
    double MWspeed;
    /*subsytem class declarions of the bot*/
    //custom math reference
    Math myMath;

    // upadate controller vars for bongo orientation
    void catieControl(double rotation)
    {
        LXaxis = (master.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
        LYaxis = (master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
        RXaxis = (master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) / rotationTune;
        RYaxis = (master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

        FLspeed = RYaxis + RXaxis;
        FRspeed = RYaxis - RXaxis;
        BLspeed = RYaxis + RXaxis;
        BRspeed = RYaxis - RXaxis;

        // set speeds in order to move bongo in target agle taken into account
        // current angle
        if (((LXaxis > deadZone) || (LXaxis < -deadZone)) ||
            ((LYaxis > deadZone) || (LYaxis < -deadZone)))
        {
            // updates speed
            double speed = myMath.TwoPointsDistance(0, 0, LXaxis, LYaxis);

            // find the angle between straight forward
            double Dangle = acos(LYaxis / speed) * 180 / M_PI;

            // negative if the x axis is on left or neg
            if (LXaxis < 0)
            {
                Dangle *= -1;
            }

            // current angle
            double currentAngle = rotation;
            FLspeed += myMath.sRound(
                myMath.multiplier(FLnum, currentAngle, Dangle) * speed, 3);
            FRspeed += myMath.sRound(
                myMath.multiplier(FRnum, currentAngle, Dangle) * speed, 3);
            BLspeed += myMath.sRound(
                myMath.multiplier(BLnum, currentAngle, Dangle) * speed, 3);
            BRspeed += myMath.sRound(
                myMath.multiplier(BRnum, currentAngle, Dangle) * speed, 3);
        }

        //under is the number in the denominator of the largest number when it equals 100
        //scales down other numbers relative to it
        double under = myMath.greatest(fabs(FLspeed), fabs(FRspeed), fabs(BLspeed),
                                       fabs(BRspeed)) /
                       100;
        FLspeed = (FLspeed / under);
        FRspeed = (FRspeed / under);
        BLspeed = (BLspeed / under);
        BRspeed = (BRspeed / under);
    }

    // upadate controller vars for bongo orientation
    //AKA tyler drive
    void tylerControl(double LX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X), 
                        double LY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
                        double RX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X))
    {
        LXaxis = LX;
        LYaxis = LY;
        RXaxis = RX / rotationTune;
        FLspeed = LYaxis + LXaxis + RXaxis;
        FRspeed = LYaxis - LXaxis - RXaxis;
        BLspeed = LYaxis - LXaxis + RXaxis;
        BRspeed = LYaxis + LXaxis - RXaxis;
        MWspeed = LYaxis;

        //under is the number in the denominator of the largest number when it equals 100
        //scales down other numbers relative to it
        double under = myMath.greatest(fabs(FLspeed), fabs(FRspeed), fabs(BLspeed),
                                       fabs(BRspeed)) /
                       100;

        FLspeed = (FLspeed / under) * maxSpeedMultiplier;
        FRspeed = (FRspeed / under) * maxSpeedMultiplier;
        BLspeed = (BLspeed / under) * maxSpeedMultiplier;
        BRspeed = (BRspeed / under) * maxSpeedMultiplier;
    }

    void arcadeControl(double LX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X), 
                        double LY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
                        double RX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)){
        LXaxis = LX;
        LYaxis = LY;
        RXaxis = RX / rotationTune;
        FLspeed = LYaxis + RXaxis;
        FRspeed = LYaxis - RXaxis;
        BLspeed = LYaxis + RXaxis;
        BRspeed = LYaxis - RXaxis;
        MWspeed = LYaxis;


        //under is the number in the denominator of the largest number when it equals 100
        //scales down other numbers relative to it
        double under = myMath.greatest(fabs(FLspeed), fabs(FRspeed), fabs(BLspeed),
                                       fabs(BRspeed)) /
                       100;

        FLspeed = (FLspeed / under) * maxSpeedMultiplier;
        FRspeed = (FRspeed / under) * maxSpeedMultiplier;
        BLspeed = (BLspeed / under) * maxSpeedMultiplier;
        BRspeed = (BRspeed / under) * maxSpeedMultiplier;
    }

    // drives motors from private vars AKA refresh motor speeds
    void move()
    {
        FR.move_velocity(myMath.toRPM(false, FRspeed, FR.get_gearing()));
        FL.move_velocity(myMath.toRPM(false, FLspeed, FL.get_gearing()));
        BR.move_velocity(myMath.toRPM(false, BRspeed, BR.get_gearing()));
        BL.move_velocity(myMath.toRPM(false, BLspeed, BL.get_gearing()));
        MW.move_velocity(myMath.toRPM(false, MWspeed, MW.get_gearing()));
    }
    
    void moveVolt(){
        FR.move_voltage(myMath.toVolt(false, FRspeed));
        FL.move_voltage(myMath.toVolt(false, FLspeed));
        BR.move_voltage(myMath.toVolt(false, BRspeed));
        BL.move_voltage(myMath.toVolt(false, BLspeed));
    }

    //update left motor side of bongo
    void moveTimed(double speed, double time)
    {
        FL.move_velocity(myMath.toRPM(false, speed, FL.get_gearing()));
        BL.move_velocity(myMath.toRPM(false, speed, BL.get_gearing()));
        FR.move_velocity(myMath.toRPM(false, speed, FR.get_gearing()));
        BR.move_velocity(myMath.toRPM(false, speed, BR.get_gearing()));
        delay(time);
        FL.move_velocity(0);
        BL.move_velocity(0);
        FR.move_velocity(0);
        BR.move_velocity(0);
    }

    //add to FLspeed variable
    void addToFLspeed(double val)
    {
        FLspeed += val;
    }

    //add to FRspeed variable
    void addToFRspeed(double val)
    {
        FRspeed += val;
    }

    //add to BLspeed variable
    void addToBLspeed(double val)
    {
        BLspeed += val;
    }

    //add to BRspeed variable
    void addToBRspeed(double val)
    {
        BRspeed += val;
    }

    //update left motor side of bongo
    void moveLeft(double speed)
    {
        FL.move_velocity(myMath.toRPM(false, speed, FL.get_gearing()));
        BL.move_velocity(myMath.toRPM(false, speed, BL.get_gearing()));
    }

    //update right motor side of bongo
    void moveRight(double speed)
    {
        FR.move_velocity(myMath.toRPM(false, speed, FR.get_gearing()));
        BR.move_velocity(myMath.toRPM(false, speed, BR.get_gearing()));
    }

    //update left motor side of bongo
    void moveLeftVolt(double speed)
    {
        FL.move_voltage(speed);
        BL.move_voltage(speed);
    }

    //update right motor side of bongo
    void moveRightVolt(double speed)
    {
        FR.move_voltage(speed);
        BR.move_voltage(speed);
    }

    //individually set FL motor speed;
    void moveFL(double speed)
    {
        FL.move_velocity(myMath.toRPM(false, speed, FL.get_gearing()));
    }

    //individually set FR motor speed;
    void moveFR(double speed)
    {
        FR.move_velocity(myMath.toRPM(false, speed, FR.get_gearing()));
    }

    //individually set BL motor speed;
    void moveBL(double speed)
    {
        BL.move_velocity(myMath.toRPM(false, speed, BL.get_gearing()));
    }

    //individually set BR motor speed;
    void moveBR(double speed)
    {
        BR.move_velocity(myMath.toRPM(false, speed, BR.get_gearing()));
    }

    // stops all movement
    void stopAll()
    {
        moveLeft(0);
        moveRight(0);
    }
};
#endif // ifndef MOVE
