#if 1
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

void opcontrol()
{
    //space to run stuff once before begin driving
    ADIDigitalOut clawLock('E');
    master.print(0, 8, "Player 1");
    while (true)
    {
        //prints to screen the position and rotation of bongo
        Bongo.debugPos();

        // tyler control
        Bongo.tylerControl();
        // catie control
        //Bongo.catieControl();

        //manual powering 
        if (master.get_digital(DIGITAL_L2)){
            Bongo.Lift.liftDown(); //manually pushes arm down at max torque
        } else if (master.get_digital(DIGITAL_L1)){
            Bongo.Lift.liftUp(); //manually lifts arm with max torque
        } else {
            Bongo.Lift.stopAll(); //stops manual controll
        }

        if (master.get_digital(DIGITAL_R2)){
            Bongo.Lift.clawDown(); //manually pushes arm down at max torque
        } else if (master.get_digital(DIGITAL_R1)){
            Bongo.Lift.clawUp(); //manually lifts arm with max torque
        } else {
            Bongo.Lift.stopClaw(); //stops manual controll
        }

        if (master.get_digital(DIGITAL_A)){
            clawLock.set_value(HIGH);;
        } else if (master.get_digital(DIGITAL_Y)){
            clawLock.set_value(LOW);
        }

        // starts the spin on motors or cuts power
        Bongo.Movement.move();
        //delay between updates
        delay(20);
    }
}
#endif