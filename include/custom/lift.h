#ifndef LIFT
#define LIFT
#include "math.h"
class LiftClass
{
private:
    double minLiftSpeed = 50;
    double maxLiftSpeed = 100;

    double minClawSpeed = 50; 
    double maxClawSpeed = 80; 

    double naturalLeftArmVal = 106;
    double naturalRightArmVal = 129;
    double naturalDifference = fabs(naturalLeftArmVal - naturalRightArmVal);

    double rightPotMax = 4030;
    double rightPotMin = naturalRightArmVal;

    double leftPotMax = 3755;
    double leftPotMin = naturalLeftArmVal;

    double offset = 50;

    double defaultClawSpeed = 100;
    double liftSpeed = 50;

    double clawSpeed = 0;
    double RarmLiftSpeed = 0;
    double LarmLiftSpeed = 0;

    double Pval = .1;
    double Dval = 0;

    double armMaxTorque = 1.05;
    double clawMaxTorque = 2.1;

    double ratio = -0.26; //1134 / 3951;
    bool autoLevel = true;
    double CPval = 2;
    double clawOffset = 0;
public:
    Math myMath;
    //stops motor
    void stopAll()
    {
        LarmLiftSpeed = 0;
        RarmLiftSpeed = 0;
        Larm.move_velocity(0);
        Rarm.move_velocity(0);
    }

    double getLeftMovedPot(){
        return leftArmPot.get_value() - naturalLeftArmVal;
    }

    double getRightMovedPot(){
        return rightArmPot.get_value() - naturalRightArmVal;
    }

    //void level(){
    //    double error = getRightMovedPot() - getLeftMovedPot();
    //    LarmLiftSpeed -= error * Pval;
    //    RarmLiftSpeed += error * Pval;
    //} 1134 degrees / 3951 analog vals

    void toggleAutoLevel(){
        autoLevel = !autoLevel;
    }

    void setAutoLevel(bool val){
        autoLevel = val;
    }

    double autoLevelClaw(){
        double target = (rightArmPot.get_value() - naturalRightArmVal) * ratio + clawOffset;
        double error = target - Claw.get_position();
        return error * CPval;
    }

    void update(){
        //level();
        if(autoLevel){
            Claw.move_velocity(autoLevelClaw());
        } else {
            Claw.move_velocity(clawSpeed);
        }

        if(RarmLiftSpeed <= 0 && rightArmPot.get_value() >= rightPotMin + offset){
          Rarm.move_velocity(RarmLiftSpeed);  
        } else if (RarmLiftSpeed >= 0 && rightArmPot.get_value() <= rightPotMax - offset){
            Rarm.move_velocity(RarmLiftSpeed);
        } else {
            Rarm.move_velocity(0);
        }
        if(LarmLiftSpeed <= 0 && leftArmPot.get_value() >= leftPotMin + offset){
            Larm.move_velocity(LarmLiftSpeed);
        } else if (LarmLiftSpeed >= 0 && leftArmPot.get_value() <= leftPotMax - offset){
            Larm.move_velocity(LarmLiftSpeed);
        } else {
            Larm.move_velocity(0);
        }
        //std::string firstLine = "L:" + std::__cxx11::to_string(LarmLiftSpeed) + "_R:" + std::__cxx11::to_string(RarmLiftSpeed);
        //master.set_text(1, 1, firstLine);
    }

    void stopClaw()
    {
        clawSpeed = 0;
    }

    double torqueLimiter(double maxTourque, double currentTorque, double min, double max){
        return max - (max - min) * (currentTorque/maxTourque);
    }

    void liftUp(){
        /*
        LarmLiftSpeed = torqueLimiter(armMaxTorque, Larm.get_torque(), minLiftSpeed, maxLiftSpeed);
        RarmLiftSpeed = torqueLimiter(armMaxTorque, Rarm.get_torque(), minLiftSpeed, maxLiftSpeed);
        */
       LarmLiftSpeed = liftSpeed;
       RarmLiftSpeed = liftSpeed;

    }

    void liftDown(){
        /*
        LarmLiftSpeed = -1 * torqueLimiter(armMaxTorque, Larm.get_torque(), minLiftSpeed, maxLiftSpeed);
        RarmLiftSpeed = -1 * torqueLimiter(armMaxTorque, Rarm.get_torque(), minLiftSpeed, maxLiftSpeed);
        */
       LarmLiftSpeed = -liftSpeed;
       RarmLiftSpeed = -liftSpeed;
    }

    void clawUp(){
        //clawSpeed = torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed);
        clawSpeed = defaultClawSpeed;
    }

    void clawDown(){
        //clawSpeed = -1 * torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed);
        clawSpeed = -defaultClawSpeed;
    }


};
#endif // ifndef LIFT
