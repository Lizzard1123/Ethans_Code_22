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


bool recording = false; // if the robot is recording data
bool hasItRecorded = false;
bool ready = false; // storage array ready ( full of zeros )

FILE* sd;

int currentDataLine = 0;

const int recordTime = 60; // in seconds
const int maxDataLength = (recordTime * 1000) / driverSpeed;
const int maxSegmentLength = MaxRecords;
double replayData[maxDataLength][maxSegmentLength]; 

//set all in replayData to 0
void fillEmpty(){ 
    printf("attempting to fill array \n");
    if(!ready){
        for(int i = 0; i < maxDataLength; i++){
            for(int j = 0; j < maxSegmentLength; j++){
                replayData[i][j] = 0;
            }
        }
    }
    ready = true;
    printf("Done filling\n");
}

//sets current recording line of replayData to controller inputs + others
void setData(bool toSd = false){
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

void setDataToSd(){
    if(toSd){
        if(sd == NULL){
            printf("Error opening sd");
        } else {
                double lineData[maxRecords] = {0};
                //TODO replace below with lineData and get rid of [currentDataLine]
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
}

//finishes recording for this itteration of data
void finalizeData(){
    if(recording){
        currentDataLine = currentDataLine + 1;
        //printf("Filling data line: %f\n", currentDataLine);
        if(currentDataLine == (int) (maxDataLength / 4)){
            printf("25%\n");
        }
        if(currentDataLine == (int) (maxDataLength / 2)){
            printf("50%\n");
        }
        if(currentDataLine == (int) (3 * maxDataLength / 4)){
            printf("75%\n");
        }
        if(currentDataLine == (int) (maxDataLength)){
            printf("100%\n");
            printf("stopping \n");
            stopRecording();
        }
    }
}

void startRecording(){
    printf("Starting recording\n");
    setRecording(true);
    hasItRecorded = true;
    fillEmpty();
}

bool hasRecorded(){
    return hasItRecorded;
}
//stop the recording + post Processing
void stopRecording(){
    //post processing 
    //double speed half time
    printf("Done Recording -> plug into terminal for transfer \n");
    recording = false;
    Bongo.Lift.stopAll();
    Bongo.Movement.stopAll();
    Bongo.Pneumatics.setRingles(false);
}

void printUnfilteredData(){
    printf("Printing Data -- Data Length: %f\n", (double) maxDataLength);
    recording = false;
    printf("{");
    for(int i = 0; i < maxDataLength; i++){ //every segment
        printf("{");
        for(int j = 0; j < maxSegmentLength; j++){ //every input
            if(j == MaxRecords - 1){
                printf("%f", (double) replayData[i][j]);
            } else {
                printf("%f,", (double) replayData[i][j]);
            }
            delay(5);
        }
        
        //num represents how many segments before newline
        double calcTime = ((((double)i+2)/maxDataLength)*recordTime);
        if(i == maxDataLength){
            printf("}");
            printf("/*Time: %f */", calcTime);
            printf("\n");
        } else {
            if(i % 1 == 0){
                printf("},");
                printf("/*Time: %f */", calcTime);
                printf("\n");
            } else {
                printf("},");
                printf("/*Time: %f */", calcTime);
            }
        }
    }
    printf("}\n");
    printf("Printing Data Done\n");
    printf("Data Length: %f\n", (double)maxDataLength);
}

void printData(){
    //Sparse Array algo
    recording = false;
    printf("Starting in 3s\n");
    delay(3000);
    printf("Starting Sparse Data Algo\n");
    int dataLength = 0;
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(fabs(replayData[i][j]) >= 1 && fabs(replayData[i][j]) <= 10000){
                dataLength++;
            }
        }
    }
    printf("dataLength: %f\n", (double)dataLength);
    int currentDataPoint = 0;
    double sparseArray[dataLength][3];
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(fabs(replayData[i][j]) >= 1 && fabs(replayData[i][j]) <= 10000){
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
                printf("%f", (double) sparseArray[i][j]);
            } else {
                printf("%f,", (double) sparseArray[i][j]);
            }
            delay(20);
        }
        printf("},");
        //printf("/*Time: %f */", (((double)sparseArray[i][2]+1)/maxDataLength)*recordTime);
        printf("\n");
    }
    printf("}\n");
    printf("Sparse Data length: %f\n", (double)dataLength);
}

void printDataToSD(){
    //Sparse Array algo
    recording = false;
    printf("Printing to SD in 3s\n");
    delay(3000);
    printf("Starting Sparse Data Algo\n");
    int dataLength = 0;
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(fabs(replayData[i][j]) >= 1 && fabs(replayData[i][j]) <= 10000){
                dataLength++;
            }
        }
    }
    printf("dataLength: %f\n", (double)dataLength);
    int currentDataPoint = 0;
    double sparseArray[dataLength][3];
    for(int i = 0; i < maxDataLength; i++){
        for(int j = 0; j < maxSegmentLength; j++){
            if(fabs(replayData[i][j]) >= 1 && fabs(replayData[i][j]) <= 10000){
                sparseArray[currentDataPoint][0] = replayData[i][j]; // first element is data
                sparseArray[currentDataPoint][1] = j; // second is the encoded num
                sparseArray[currentDataPoint][2] = i; //third is the time (line num)
                currentDataPoint++;
            }
        }
    }

    FILE* sd = fopen("/usd/RecordedData.txt", "w");
    if(sd == NULL){
        printf("Error opening SD\n");
    } else {
        printf("Printing Sparse Data to SD\n");
        fprintf(sd, "{");
        for(int i = 0; i < dataLength; i++){ //every segment
            fprintf(sd, "{");
            for(int j = 0; j < 3; j++){ //every input
                if(j == 2){
                    fprintf(sd, "%f", (double) sparseArray[i][j]);
                } else {
                    fprintf(sd, "%f,", (double) sparseArray[i][j]);
                }
                //delay(5);
            }
            fprintf(sd, "},");
             //printf("/*Time: %f */", (((double)sparseArray[i][2]+1)/maxDataLength)*recordTime);
            fprintf(sd, "\n");
        }
        fprintf(sd, "}\n");
        fprintf(sd, "Sparse Data length: %f\n", (double)dataLength);
        printf("Done with SD; closing\n");
        fclose(sd);
    }
}

//returns last index, runs motor values 
void runSegment(double dataToBeReplayed[][3], int dataLength, int timeToRun){
    //dataLine for segment
    double dataLine[MaxRecords];
    //fill to zeros
    for(int i = 0; i < MaxRecords; i++){
        dataLine[MaxRecords] = 0;
    }
    for(int i = 0; i < dataLength; i++){
        if((int)dataToBeReplayed[i][2] == timeToRun){
            printf("found on line: %i\n", i);
            printf("number: %i\n", (int)dataToBeReplayed[i][1]);
            printf("val: %f\n", dataToBeReplayed[i][0]);
            dataLine[(int)dataToBeReplayed[i][1]] = dataToBeReplayed[i][0];
            printf("check: %f\n", dataLine[(int)dataToBeReplayed[i][1]]);
        }
    }

    //DRIVE CODE

    Bongo.Movement.tylerControl(dataLine[LXDataNum],dataLine[LYDataNum],dataLine[RXDataNum]);

    //manual powering 
    if ((int)dataLine[L2ButtonDigital]){
        Bongo.Lift.liftDown(); //manually pushes arm down at max torque
    } else if ((int)dataLine[L1ButtonDigital]){
        Bongo.Lift.liftUp(); //manually lifts arm with max torque
    } else {
        Bongo.Lift.stopArm(); //stops manual controll
    }

    if ((int)dataLine[R2ButtonDigital]){
        Bongo.Lift.clawDown(); //manually pushes claw down at max torque
        //skillsLift.setAutoLevel(false);
    } else if ((int)dataLine[R1ButtonDigital]){
        Bongo.Lift.clawUp(); //manually lifts claw with max torque
        //skillsLift.setAutoLevel(false);
    } else {
        Bongo.Lift.stopClaw(); //stops manual controll
    }

    if ((int)dataLine[AButtonPress]){
        Bongo.Pneumatics.toggleClaw();
    }
    if ((int)dataLine[XButtonPress]){
        Bongo.Pneumatics.toggletilt();
    }
    if ((int)dataLine[YButtonPress]){
        Bongo.Pneumatics.toggleBack();
    }
    if ((int)dataLine[downArrowPress]){
        RingleLift.move_relative(-360, 50);
    }
    if((int)dataLine[upArrowPress]){
        Bongo.Pneumatics.toggleRingles();
    }

    Bongo.Movement.move();

}

void executeData(double dataToBeReplayed[][3], int dataLength, int dataTime){
    printf("Executing Data\n");
    dataTime = (dataTime * 1000) / driverSpeed;
    printf("Total Data time: %f\n", (double)dataTime);
    for(int i = 0; i < dataTime; i++){
        printf("running line: %f\n", (double)i);
        printf("total: %f\n", (double)dataLength);
        runSegment(dataToBeReplayed, dataLength, i); //similate inputs 
        delay(driverSpeed / (speedUp ? 2 : 1)); // NEEDS to be the same as driver collected dataLine
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
