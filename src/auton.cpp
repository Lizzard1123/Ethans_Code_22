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


bool recording = false;
bool ready = false;
bool speedUp = false;


int currentDataLine = 0;
//skills
const int recordTime = 120; // in seconds
const int maxDataLength = (recordTime * 1000) / driverSpeed;
const int maxSegmentLength = MaxRecords;
double replayData[maxDataLength][maxSegmentLength]; 

void fillEmpty(){ //set all to 0
    printf("attempting to fill array \n");
    if(!ready){
        for(int i = 0; i < maxDataLength; i++){
            for(int j = 0; j < maxSegmentLength; j++){
                replayData[i][j] = 0;
            }
        }
    }
    ready = true;
    printf("Done filling");
}

void setData(){
    if(recording && !ready){ // check to make sure that it is empty
        fillEmpty();
        printf("Had to catch itself bc it wasnt ready");
    } 
    if(recording){
        replayData[currentDataLine][LXDataNum] = master.get_analog(ANALOG_LEFT_X);
        replayData[currentDataLine][LYDataNum] = master.get_analog(ANALOG_LEFT_Y);
        replayData[currentDataLine][RXDataNum] = master.get_analog(ANALOG_RIGHT_X);
        replayData[currentDataLine][RYDataNum] = master.get_analog(ANALOG_RIGHT_Y);
        replayData[currentDataLine][upArrowDigital] = master.get_digital(DIGITAL_UP);
        replayData[currentDataLine][rightArrowDigital] = master.get_digital(DIGITAL_RIGHT);
        replayData[currentDataLine][downArrowDigital] = master.get_digital(DIGITAL_DOWN);
        replayData[currentDataLine][leftArrowDigital] = master.get_digital(DIGITAL_LEFT);
        replayData[currentDataLine][upArrowPress] = master.get_digital_new_press(DIGITAL_UP);
        replayData[currentDataLine][rightArrowPress] = master.get_digital_new_press(DIGITAL_RIGHT);
        replayData[currentDataLine][downArrowPress] = master.get_digital_new_press(DIGITAL_DOWN);
        replayData[currentDataLine][leftArrowPress] = master.get_digital_new_press(DIGITAL_LEFT);
        replayData[currentDataLine][XButtonDigital] = master.get_digital(DIGITAL_X);
        replayData[currentDataLine][AButtonDigital] = master.get_digital(DIGITAL_A);
        replayData[currentDataLine][BButtonDigital] = master.get_digital(DIGITAL_B);
        replayData[currentDataLine][YButtonDigital] = master.get_digital(DIGITAL_Y);
        replayData[currentDataLine][XButtonPress] = master.get_digital_new_press(DIGITAL_X);
        replayData[currentDataLine][AButtonPress] = master.get_digital_new_press(DIGITAL_A);
        replayData[currentDataLine][BButtonPress] = master.get_digital_new_press(DIGITAL_B);
        replayData[currentDataLine][YButtonPress] = master.get_digital_new_press(DIGITAL_Y);
        replayData[currentDataLine][L1ButtonDigital] = master.get_digital(DIGITAL_L1);
        replayData[currentDataLine][L2ButtonDigital] = master.get_digital(DIGITAL_L2);
        replayData[currentDataLine][R1ButtonDigital] = master.get_digital(DIGITAL_R1);
        replayData[currentDataLine][R2ButtonDigital] = master.get_digital(DIGITAL_R2);
        replayData[currentDataLine][L1ButtonPress] = master.get_digital_new_press(DIGITAL_L1);
        replayData[currentDataLine][L2ButtonPress] = master.get_digital_new_press(DIGITAL_L2);
        replayData[currentDataLine][R1ButtonPress] = master.get_digital_new_press(DIGITAL_R1);
        replayData[currentDataLine][R2ButtonPress] = master.get_digital_new_press(DIGITAL_R2);
        //custom
        replayData[currentDataLine][FLActualVelocity] = FL.get_actual_velocity();
        replayData[currentDataLine][FRActualVelocity] = FR.get_actual_velocity();
        replayData[currentDataLine][BLActualVelocity] = BL.get_actual_velocity();
        replayData[currentDataLine][BRActualVelocity] = BR.get_actual_velocity();
        replayData[currentDataLine][ClawActualVelocity] = Claw.get_actual_velocity();
        replayData[currentDataLine][RingleLiftActualVelocity] = RingleLift.get_actual_velocity();
        replayData[currentDataLine][LarmActualVelocity] = Larm.get_actual_velocity();
        replayData[currentDataLine][RarmActualVelocity] = Rarm.get_actual_velocity();
        replayData[currentDataLine][ClawPosition] = Claw.get_position();
        replayData[currentDataLine][RingleLiftPosition] = RingleLift.get_position();
        replayData[currentDataLine][LarmPosition] = Larm.get_position();
        replayData[currentDataLine][RarmPosition] = Rarm.get_position();
        replayData[currentDataLine][rightOdomPosition] = rightOdom.get();
        replayData[currentDataLine][leftOdomPosition] = pros::c::ext_adi_encoder_get(leftOdom);
    }
}

//finishes recording for this itteration of data
void finalizeData(){
    if(recording){
        currentDataLine++;
        //printf("Filling data line: %f\n", currentDataLine);
        if(currentDataLine == maxDataLength){
            printf("stopping");
            stopRecording();
        }
    }
}

//stop the recording + post Processing
void stopRecording(){
    //post processing 
    //double speed half time
    
    //print the unfilterd array
    //printUnfilteredData();

    //print filterd sparse array
    printData();
}

void printUnfilteredData(){
    printf("Printing Data\n");
    recording = false;
    printf("{");
    for(int i = 0; i < maxDataLength; i++){ //every segment
        printf("{");
        for(int j = 0; j < maxSegmentLength; j++){ //every input
            if(j == 15){
                printf("%f", replayData[i][j]);
            } else {
                printf("%f,", replayData[i][j]);
            }
            delay(10);
        }
        /*
        //num represents how many segments before newline
        if(i % 2 == 0){
            printf("},\n");
        } else {
            printf("},");
        }
        */
        printf("},\n");
    }
    printf("}\n");
}

void printData(){
    //Sparse Array algo
    int dataLength = 0;
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(replayData[i][j] != 0){
                dataLength++;
            }
        }
    }
    int currentDataPoint = 0;
    double sparseArray[dataLength][3];
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(replayData[i][j] != 0){
                sparseArray[currentDataPoint][0] = replayData[i][j]; // first element is data
                sparseArray[currentDataPoint][1] = j; // second is the encoded num
                sparseArray[currentDataPoint][2] = i; //third is the time (line num)
                currentDataPoint++;
            }
        }
    }

    printf("Printing Sparse Data\n");
    recording = false;
    printf("{");
    for(int i = 0; i < dataLength; i++){ //every segment
        printf("{");
        for(int j = 0; j < 3; j++){ //every input
            if(j == 2){
                printf("%f", replayData[i][j]);
            } else {
                printf("%f,", replayData[i][j]);
            }
            delay(10);
        }
        /*
        //num represents how many segments before newline
        if(i % 10 == 0){
            printf("},\n");
        } else {
            printf("},");
        }
        */
        printf("},\n");
    }
    printf("}\n");
    printf("Sparse Data length: %f\n", dataLength);
}

//returns last index, runs motor values 
int runSegment(double dataToBeReplayed[][3], int dataLength, int startIndex){
    //dataLine for segment
    double dataLine[MaxRecords];
    int endIndex = dataLength;
    //fill to zeros
    for(int i = 0; i < MaxRecords; i++){
        dataLine[MaxRecords] = 0;
    }
    //find data to fill
    double currentTime = dataToBeReplayed[startIndex][2];
    for(int i = 0; i < dataLength-startIndex; i++){
        if(dataToBeReplayed[startIndex + i][2] != currentTime){
            endIndex = startIndex + i;
        } else { //still on same time keep filling data
            dataLine[(int)dataToBeReplayed[startIndex + i][1]] = dataToBeReplayed[startIndex + i][0];
        }
    }


    //DRIVE CODE

    Bongo.Movement.tylerControl(dataLine[LXDataNum],dataLine[LYDataNum],dataLine[RXDataNum]);

    //manual powering 
    if (dataLine[L2ButtonDigital]){
        Bongo.Lift.liftDown(); //manually pushes arm down at max torque
    } else if (dataLine[L1ButtonDigital]){
        Bongo.Lift.liftUp(); //manually lifts arm with max torque
    } else {
        Bongo.Lift.stopArm(); //stops manual controll
    }

    if (dataLine[R2ButtonDigital]){
        Bongo.Lift.clawDown(); //manually pushes claw down at max torque
        //skillsLift.setAutoLevel(false);
    } else if (dataLine[R1ButtonDigital]){
        Bongo.Lift.clawUp(); //manually lifts claw with max torque
        //skillsLift.setAutoLevel(false);
    } else {
        Bongo.Lift.stopClaw(); //stops manual controll
    }

    if (dataLine[AButtonPress]){
        Bongo.Pneumatics.toggleClaw();
    }
    if (dataLine[XButtonPress]){
        Bongo.Pneumatics.toggletilt();
    }
    if (dataLine[YButtonPress]){
        Bongo.Pneumatics.toggleBack();
    }
    if (dataLine[downArrowPress]){
        RingleLift.move_relative(-360, 50);
    }
    if(dataLine[upArrowPress]){
        Bongo.Pneumatics.toggleRingles();
    }

    Bongo.Movement.move();

    return endIndex;
}

void executeData(double dataToBeReplayed[][3], int dataLength){
    printf("Executing Skills Data");
    for(int i = 0; i < dataLength; i++){
       i = runSegment(dataToBeReplayed, dataLength, i); //similate inputs 
       delay(driverSpeed / speedUp ? 2 : 1); // NEEDS to be the same as driver collected dataLine
    }
}


bool isRecording(){
  return recording;
}

void setRecording(bool val){
  recording = val;
}

bool isSpeedUp(){
  return speedUp;
}

void setSpeedUp(bool val){
  speedUp = val;
}


void autonomous()
{
  Bongo.autonomous();
}
