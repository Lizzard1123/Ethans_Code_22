#ifndef LIFT
#define LIFT
#include "math.h"
#include "auton.h"
class LiftClass
{
private:
    double minLiftSpeed = 50;
    double maxLiftSpeed = 100;

    double minClawSpeed = 40; 
    double maxClawSpeed = 80; 

    double defaultClawSpeed = 100;
    double liftSpeed = 50;

    double clawSpeed = 0;
    double RarmLiftSpeed = 0;
    double LarmLiftSpeed = 0;

    double Pval = .1;

    double armMaxTorque = 1.05;
    double clawMaxTorque = 2.1;

    bool autoLevel = false;
    double CPval = 3;
    double clawOffset = 0;
    double clawIncrease = 6;

    double moveClawPval = 25;
    double moveArmPval = 100;

    bool PIDControl = false;
    
public:
    Math myMath;
    double leftMax = -1651;
    double rightMax = -1643;
    double leftMin = -214;
    double rightMin = -203;

    double ClawError = 0;
    double LarmError = 0;
    double RarmError = 0;

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

    void toggleAutoLevel(){
        //autoLevel = !autoLevel;
        clawOffset = 0;
    }

    void setAutoLevel(bool val){
        autoLevel = val;
    }
    
    double autoLevelClaw(){
        double error = clawOffset - Wrist.get_pitch();
        return error * CPval;
    }

    void setPID(double dataToBeReplayed[MaxRecords], double futureDataToBeReplayed[MaxRecords]){
        PIDControl = true;
        //left Arm
        Larm.move_voltage(dataToBeReplayed[LarmVolt] + (futureDataToBeReplayed[LarmPosition] - Larm.get_position()) * moveArmPval);
        //right Arm
        Rarm.move_voltage(dataToBeReplayed[RarmVolt] + (futureDataToBeReplayed[RarmPosition] - Rarm.get_position()) * moveArmPval);
        //Claw
        Claw.move_voltage(dataToBeReplayed[ClawVolt] + (futureDataToBeReplayed[ClawPosition] - Claw.get_position()) * moveClawPval);

        LarmError += dataToBeReplayed[LarmPosition] - Larm.get_position();
        RarmError += dataToBeReplayed[RarmPosition] - Rarm.get_position();
        ClawError += dataToBeReplayed[ClawPosition] + Claw.get_position();
    }
    
    void stopPIDArm(){
        PIDControl = false;
    }

    void update(){
        //level();
        //if(autoLevel){
        //    Claw.move_velocity(autoLevelClaw());
        //} else {
        //    Claw.move_velocity(clawSpeed);
        //}
        if(!PIDControl){
            if(Larm.get_position() >= 0 && LarmLiftSpeed < 0){
                Larm.move_velocity(LarmLiftSpeed);
            } else if (RarmLiftSpeed > 0){
                Larm.move_velocity(LarmLiftSpeed);
            } else {
                Larm.move_velocity(0);
            }
            if(Rarm.get_position() >= 0 && RarmLiftSpeed < 0){
                Rarm.move_velocity(RarmLiftSpeed);
            } else if (RarmLiftSpeed > 0){
                Rarm.move_velocity(RarmLiftSpeed); 
            } else {
                Rarm.move_velocity(0);
            }
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
        if(autoLevel){
            if(clawOffset <= 70){
                clawOffset+=clawIncrease;
            }
        } else {
            clawSpeed = torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed); 
        }
        //clawSpeed = defaultClawSpeed;
        //Claw.move_velocity(clawSpeed);
    }
 
    void clawDown(){
        if(autoLevel){
            if(clawOffset >= -80){
                clawOffset-=clawIncrease;
            }
        } else {
            clawSpeed = -1 * torqueLimiter(clawMaxTorque, Claw.get_torque(), minClawSpeed, maxClawSpeed);
        }
        //clawSpeed = -defaultClawSpeed;
        //Claw.move_velocity(clawSpeed);
    }


};
#endif // ifndef LIFT
