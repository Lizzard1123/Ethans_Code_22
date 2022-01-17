#include "custom/robot.h"

Robot Bongo;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


bool recording = true;
bool ready = false;
int dataSize = 0;
//skills
const int dataLength = 3500;
const int segmentLength = 16;
double replayData[dataLength][segmentLength]; //16 should stay 3500 should be a lil lower and = to dataSize
/*
0 - LX joystick
1 - LY joystick
2 - RX joystick
3 - RY joystick

4 - upArrow (0 nothing, 1 trigger)
5 - rightArrow (0 nothing, 1 trigger)
6 - downArrow (0 nothing, 1 trigger)
7 - leftArrow (0 nothing, 1 trigger)

8 - X (0 nothing, 1 trigger)
9 - A (0 nothing, 1 trigger)
10 - B (0 nothing, 1 trigger)
11 - Y (0 nothing, 1 trigger)

12 - L1 (0 nothing, 1 trigger)
13 - L2 (0 nothing, 1 trigger)
14 - R1 (0 nothing, 1 trigger)
15 - R2 (0 nothing, 1 trigger)
*/

void fillEmpty(){ //set all to 0
    printf("attempting to fill array \n");
    if(!ready){
        for(int i = 0; i < dataLength; i++){
            for(int j = 0; j < segmentLength; j++){
                replayData[i][j] = 0;
            }
        }
    }
    ready = true;
}

void setData(int num, double val){
    if(recording && !ready){
        fillEmpty();
    } 
    if(recording){
        replayData[dataSize][num] = val;
    }
}

void finalizeData(){
    if(recording){
        dataSize++;
        printf("Filling data line: %f\n", dataSize);
    }
}

void printData(){
    printf("Printing Data\n");
    recording = false;
    printf("{");
    for(int i = 0; i < dataLength; i++){ //every segment
    printf("{");
        for(int j = 0; j < segmentLength; j++){ //every input
            printf("%f,", replayData[i][j]);
        }
    printf("},\n");
    }
    printf("}\n");
}

void runSegment(int line){
    //replayData[line] for segment

    Bongo.Movement.tylerControl(replayData[line][0],replayData[line][1],replayData[line][2]);

    //manual powering 
    if (replayData[line][13]){
        Bongo.Lift.liftDown(); //manually pushes arm down at max torque
    } else if (replayData[line][12]){
        Bongo.Lift.liftUp(); //manually lifts arm with max torque
    } else {
        Bongo.Lift.stopArm(); //stops manual controll
    }

    if (replayData[line][15]){
        Bongo.Lift.clawDown(); //manually pushes claw down at max torque
        //skillsLift.setAutoLevel(false);
    } else if (replayData[line][14]){
        Bongo.Lift.clawUp(); //manually lifts claw with max torque
        //skillsLift.setAutoLevel(false);
    } else {
        Bongo.Lift.stopClaw(); //stops manual controll
    }

    if (replayData[line][9]){
        Bongo.Pneumatics.toggleClaw();
    }
    if (replayData[line][8]){
        Bongo.Pneumatics.toggletilt();
    }
    if (replayData[line][11]){
        Bongo.Pneumatics.toggleBack();
    }
    if (replayData[line][6]){
        Lift.move_relative(-360, 50);
    }
    if(replayData[line][4]){
        Bongo.Pneumatics.toggleRingles();
    }

    Bongo.Movement.move();
}

void executeSkillsData(){
    printf("Executing Skills Data");
    for(int i = 0; i < dataLength; i++){
       runSegment(i); //similate inputs 
       delay(20); // NEEDS to be the same as driver collected replayData
    }
}

bool isRecording(){
  return recording;
}

void setRecording(bool val){
  recording = val;
}

void autonomous()
{
  Bongo.autonomous();
}
