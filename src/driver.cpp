
#include "custom/robot.h"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 //test from phone v2 for streaks lol again, schoolwork, i need to be leetcoding
int count = 0;

void opcontrol()
{
    //space to run stuff once before begin driving
    //ADIDigitalOut clawLock('E');
    //ADIDigitalOut tiltLock('G');
    //ADIDigitalOut backLock('H');
    master.print(0, 8, "Player 1");
    Bongo.Lift.stopPIDArm();
    //startRecording();
    while (true)
    {//
        //
        if(master.get_digital_new_press(DIGITAL_LEFT)){
            if(!hasRecorded()){
               startRecording();
            }
            //learnEncoder(skills_Data, skills_dataLength, skillsDataTime);
            //Bongo.autonomous();
        }
        //prints to screen the position and rotation of bongo
        Bongo.debugPos();


        //Control types
        // tyler control
        Bongo.tylerControl();

        //arcade control
        //Bongo.arcadeControl();

        // catie control
        //Bongo.catieControl();

        //manual powering 
        if (master.get_digital(DIGITAL_L2)){
            Bongo.Lift.liftDown(); //manually pushes arm down at max torque
        } else if (master.get_digital(DIGITAL_L1)){
            Bongo.Lift.liftUp(); //manually lifts arm with max torque
        } else {
            Bongo.Lift.stopArm(); //stops manual controll
        }

        if (master.get_digital(DIGITAL_R2)){
            Bongo.Lift.clawDown(); //manually pushes claw down at max torque
            //Bongo.Lift.setAutoLevel(false);
        } else if (master.get_digital(DIGITAL_R1)){
            Bongo.Lift.clawUp(); //manually lifts claw with max torque
            //Bongo.Lift.setAutoLevel(false);
        } else {
            Bongo.Lift.stopClaw(); //stops manual controll
        }

        //Toggle claw
        if (master.get_digital_new_press(DIGITAL_A)){
            Bongo.Pneumatics.toggleClaw();
        }

        //Toggle tilt
        if (master.get_digital_new_press(DIGITAL_X)){
            Bongo.Pneumatics.toggletilt();
        }

        //Toggle back
        if (master.get_digital_new_press(DIGITAL_Y)){
            Bongo.Pneumatics.toggleBack();
        }

        //Toggle level
        if (master.get_digital_new_press(DIGITAL_RIGHT)){
            Bongo.Lift.toggleAutoLevel();
        }

        //if (master.get_digital_new_press(DIGITAL_DOWN)){
        //    //RingleLift.move_relative(-360, 50);
        //    //Bongo.testOdom();
        //}

        if(master.get_digital(DIGITAL_UP)){
            //Bongo.Pneumatics.toggleRingles();
            //Bongo.testOdom2();
            Bongo.Pneumatics.shiftWheelUp();
        }

        if(master.get_digital(DIGITAL_DOWN)){
            //Bongo.Pneumatics.toggleRingles();
            //Bongo.testOdom2();
            Bongo.Pneumatics.shiftWheelDown();
        }

        //if(master.get_digital_new_press(DIGITAL_RIGHT)){
        //    //Bongo.setRotation(0);
        //    //Bongo.testOdom3();
        //    Bongo.PIDClimb();
        //}

        // starts the spin on motors or cuts power
        Bongo.Movement.move();
        //Bongo.Movement.moveVolt();
        //delay between updates

        setDataToSd();
        finalizeData();
        //FL:FR:BL:BR:Rarm:Larm:Claw
        if(count%2 == 0){
            std::string values = std::__cxx11::to_string(int(FL.get_temperature())) + ":" +  std::__cxx11::to_string(int(FR.get_temperature())) + ":" +  std::__cxx11::to_string(int(BL.get_temperature())) + ":" +  std::__cxx11::to_string(int(BR.get_temperature())) + ":" +  std::__cxx11::to_string(int(Rarm.get_temperature())) + ":" +  std::__cxx11::to_string(int(Larm.get_temperature())) + ":" + std::__cxx11::to_string(int(Claw.get_temperature()));
            master.set_text(0, 0, values);
        } else if(count%3 == 1){
            std::string odomvalues = std::__cxx11::to_string(int(leftOdom.get())) + ":" +  std::__cxx11::to_string(int(rightOdom.get()));
            master.set_text(1, 0, odomvalues);
        } else {
            master.clear();
        }
        count++;
        
        
        delay(driverSpeed);
        
    }
}