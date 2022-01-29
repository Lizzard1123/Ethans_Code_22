#ifndef LIFT
#define LIFT
#include "math.h"
class LiftClass
{
private:
    double minLiftSpeed = 50;
    double maxLiftSpeed = 100 * maxSpeedMultiplier;

    double minClawSpeed = 40; 
    double maxClawSpeed = 80 * maxSpeedMultiplier; 

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
    bool autoLevel = false;
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
        clawSpeed = 0;
        Claw.move_velocity(0);
    }

    void stopArm(){
        LarmLiftSpeed = 0;
        RarmLiftSpeed = 0;
        Larm.move_velocity(0);
        Rarm.move_velocity(0);
    }

    //void level(){
    //    double error = getRightMoved() - getLeftMoved();
    //    LarmLiftSpeed -= error * Pval;
    //    RarmLiftSpeed += error * Pval;
    //} 1134 degrees / 3951 analog vals

    void toggleAutoLevel(){
        autoLevel = !autoLevel;
    }

    void setAutoLevel(bool val){
        autoLevel = val;
    }
    /*
    double autoLevelClaw(){
        double target = (Rarm.get_position() - liftMinPos) * ratio + clawOffset;
        double error = target - Claw.get_position();
        return error * CPval;
    }
    */

    void update(){
        //level();
        //if(autoLevel){
        //    Claw.move_velocity(autoLevelClaw());
        //} else {
        //Claw.move_velocity(clawSpeed);
        //}

        if(!leftSwitch.get_value() && LarmLiftSpeed < 0){
            Larm.move_velocity(LarmLiftSpeed);
            Rarm.move_velocity(RarmLiftSpeed);
        } else if (RarmLiftSpeed > 0){
            Larm.move_velocity(LarmLiftSpeed);
            Rarm.move_velocity(RarmLiftSpeed); 
        } else {
            Larm.move_velocity(0);
            Rarm.move_velocity(0);
        }
        //std::string firstLine = "L:" + std::__cxx11::to_string(int(Larm.get_position())) + "_R:" + std::__cxx11::to_string(int(Rarm.get_position())) + "_C:" + std::__cxx11::to_string(int(Claw.get_position()));
        //master.set_text(1, 1, firstLine);
    }

    void stopClaw()
    {
        Claw.move_velocity(0);
        clawSpeed = 0;
    }

    double torqueLimiter(double maxTourque, double currentTorque, double min, double max){
        return max - (max - min) * (currentTorque/maxTourque);
    }

    void liftUp(){
        
        LarmLiftSpeed = torqueLimiter(armMaxTorque, Larm.get_torque(), minLiftSpeed, maxLiftSpeed);
        RarmLiftSpeed = torqueLimiter(armMaxTorque, Rarm.get_torque(), minLiftSpeed, maxLiftSpeed);
        
       //LarmLiftSpeed = liftSpeed;
       //RarmLiftSpeed = liftSpeed;

    }

    void liftDown(){
        
        LarmLiftSpeed = -1 * torqueLimiter(armMaxTorque, Larm.get_torque(), minLiftSpeed, maxLiftSpeed);
        RarmLiftSpeed = -1 * torqueLimiter(armMaxTorque, Rarm.get_torque(), minLiftSpeed, maxLiftSpeed);

       //LarmLiftSpeed = -liftSpeed;
       //RarmLiftSpeed = -liftSpeed;
    }

    void clawUp(){
        clawSpeed = torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed);
        Claw.move_velocity(clawSpeed);
        //clawSpeed = defaultClawSpeed;
    }
 
    void clawDown(){
        clawSpeed = -1 * torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed);
        Claw.move_velocity(clawSpeed);
        //clawSpeed = -defaultClawSpeed;
    }


};
#endif // ifndef LIFT
