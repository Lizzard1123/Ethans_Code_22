#ifndef AUTON
#define AUTON

#include "global.h"
#include "lift.h"
#include "movement.h"
#include "pneumatics.h"
LiftClass skillsLift;
// class handler for movement + other funtions
RobotMovement skillsMovement;
//custom static pneumatics class
PneumaticsClass skillsPneumatics;
bool recording = true;
bool ready = false;
int dataSize = 0;
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
    }
}

void printData(){
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

    skillsMovement.tylerControl(replayData[line][0],replayData[line][1],replayData[line][2]);

    //manual powering 
    if (replayData[line][13]){
        skillsLift.liftDown(); //manually pushes arm down at max torque
    } else if (replayData[line][12]){
        skillsLift.liftUp(); //manually lifts arm with max torque
    } else {
        skillsLift.stopArm(); //stops manual controll
    }

    if (replayData[line][15]){
        skillsLift.clawDown(); //manually pushes claw down at max torque
        //skillsLift.setAutoLevel(false);
    } else if (replayData[line][14]){
        skillsLift.clawUp(); //manually lifts claw with max torque
        //skillsLift.setAutoLevel(false);
    } else {
        skillsLift.stopClaw(); //stops manual controll
    }

    if (replayData[line][9]){
        skillsPneumatics.toggleClaw();
    }
    if (replayData[line][8]){
        skillsPneumatics.toggletilt();
    }
    if (replayData[line][11]){
        skillsPneumatics.toggleBack();
    }
    if (replayData[line][6]){
        Lift.move_relative(-360, 50);
    }
    if(replayData[line][4]){
        skillsPneumatics.toggleRingles();
    }

    skillsMovement.move();
}

void executeSkillsData(){
    for(int i = 0; i < dataLength; i++){
       runSegment(i); //similate inputs 
       delay(20); // NEEDS to be the same as driver collected replayData
    }
}
#endif