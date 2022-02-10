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

FILE* sd = fopen("/usd/RecordedData.txt", "w");

int currentDataLine = 0;

const int recordTime = 16; // in seconds
int recordLength = 0;
const int maxDataLength = (recordTime * 1000) / driverSpeed;
const int maxSegmentLength = MaxRecords;
double replayData[maxDataLength][maxSegmentLength]; 

//PID movement
double lastLeftError = 0;
double lastRightError = 0;

double leftErrorScore = 0;
double rightErrorScore = 0;

double learnPval = 15;

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
        replayData[currentDataLine][leftOdomPosition] = leftOdom.get();


        replayData[currentDataLine][FLVolt] = FL.get_voltage();
        replayData[currentDataLine][FRVolt] = FR.get_voltage();
        replayData[currentDataLine][BLVolt] = BL.get_voltage();
        replayData[currentDataLine][BRVolt] = BR.get_voltage();
        replayData[currentDataLine][LarmVolt] = Larm.get_voltage();
        replayData[currentDataLine][RarmVolt] = Rarm.get_voltage();
        replayData[currentDataLine][ClawVolt] = Claw.get_voltage();
        replayData[currentDataLine][VincentRotation] = Vincent.get_rotation();
        replayData[currentDataLine][WristRotation] = Wrist.get_pitch();
        replayData[currentDataLine][VincentPitch] = Vincent.get_pitch();
        replayData[currentDataLine][recordedX] = Bongo.getX();
        replayData[currentDataLine][recordedY] = Bongo.getY();

    }
}
/*

#define FLVolt 42
#define FRVolt 43
#define BLVolt 44
#define BRVolt 45

#define VincentRotation 46
#define WristPitch 47
#define VincentYaw 48

#define recordedX 49
#define recordedY 50

*/

void setDataToSd(){
    if(recording){
        if(sd == NULL){
            printf("Error opening sd");
        } else {
            double lineData[MaxRecords] = {0};
            //TODO replace below with lineData and get rid of [currentDataLine]
            lineData[LXDataNum] = master.get_analog(ANALOG_LEFT_X);
            lineData[LYDataNum] = master.get_analog(ANALOG_LEFT_Y);
            lineData[RXDataNum] = master.get_analog(ANALOG_RIGHT_X);
            lineData[RYDataNum] = master.get_analog(ANALOG_RIGHT_Y);
            lineData[upArrowDigital] = (double)master.get_digital(DIGITAL_UP);
            lineData[rightArrowDigital] = (double)master.get_digital(DIGITAL_RIGHT);
            lineData[downArrowDigital] = (double)master.get_digital(DIGITAL_DOWN);
            lineData[leftArrowDigital] = (double)master.get_digital(DIGITAL_LEFT);
            lineData[upArrowPress] = (double)master.get_digital_new_press(DIGITAL_UP);
            lineData[rightArrowPress] = (double)master.get_digital_new_press(DIGITAL_RIGHT);
            lineData[downArrowPress] = (double)master.get_digital_new_press(DIGITAL_DOWN);
            lineData[leftArrowPress] = (double)master.get_digital_new_press(DIGITAL_LEFT);
            lineData[XButtonDigital] = (double)master.get_digital(DIGITAL_X);
            lineData[AButtonDigital] = (double)master.get_digital(DIGITAL_A);
            lineData[BButtonDigital] = (double)master.get_digital(DIGITAL_B);
            lineData[YButtonDigital] = (double)master.get_digital(DIGITAL_Y);
            lineData[XButtonPress] = (double)master.get_digital_new_press(DIGITAL_X);
            lineData[AButtonPress] = (double)master.get_digital_new_press(DIGITAL_A);
            lineData[BButtonPress] = (double)master.get_digital_new_press(DIGITAL_B);
            lineData[YButtonPress] = (double)master.get_digital_new_press(DIGITAL_Y);
            lineData[L1ButtonDigital] = (double)master.get_digital(DIGITAL_L1);
            lineData[L2ButtonDigital] = (double)master.get_digital(DIGITAL_L2);
            lineData[R1ButtonDigital] = (double)master.get_digital(DIGITAL_R1);
            lineData[R2ButtonDigital] = (double)master.get_digital(DIGITAL_R2);
            lineData[L1ButtonPress] = (double)master.get_digital_new_press(DIGITAL_L1);
            lineData[L2ButtonPress] = (double)master.get_digital_new_press(DIGITAL_L2);
            lineData[R1ButtonPress] = (double)master.get_digital_new_press(DIGITAL_R1);
            lineData[R2ButtonPress] = (double)master.get_digital_new_press(DIGITAL_R2);
            //custom
            lineData[FLActualVelocity] = FL.get_actual_velocity();
            lineData[FRActualVelocity] = FR.get_actual_velocity();
            lineData[BLActualVelocity] = BL.get_actual_velocity();
            lineData[BRActualVelocity] = BR.get_actual_velocity();
            lineData[ClawActualVelocity] = Claw.get_actual_velocity();
            lineData[RingleLiftActualVelocity] = RingleLift.get_actual_velocity();
            lineData[LarmActualVelocity] = Larm.get_actual_velocity();
            lineData[RarmActualVelocity] = Rarm.get_actual_velocity();
            lineData[ClawPosition] = Claw.get_position();
            lineData[RingleLiftPosition] = RingleLift.get_position();
            lineData[LarmPosition] = Larm.get_position();
            lineData[RarmPosition] = Rarm.get_position();
            lineData[rightOdomPosition] = rightOdom.get();
            lineData[leftOdomPosition] = leftOdom.get();
            lineData[FLVolt] = FL.get_voltage();
            lineData[FRVolt] = FR.get_voltage();
            lineData[BLVolt] = BL.get_voltage();
            lineData[BRVolt] = BR.get_voltage();
            lineData[LarmVolt] = Larm.get_voltage();
            lineData[RarmVolt] = Rarm.get_voltage();
            lineData[ClawVolt] = Claw.get_voltage();
            lineData[VincentRotation] = Vincent.get_rotation();
            lineData[WristRotation] = Wrist.get_rotation();
            lineData[VincentPitch] = Vincent.get_pitch();
            lineData[recordedX] = Bongo.getX();
            lineData[recordedY] = Bongo.getY();
            fprintf(sd, "{");
            for(int i = 0; i < MaxRecords; i++){ //every segment
                if(i == MaxRecords - 1){
                    fprintf(sd, "%f", (double) lineData[i]);
                } else {
                    fprintf(sd, "%f,", (double) lineData[i]);
                }
            }
            fprintf(sd, "},\n");
            recordLength++;
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
    fprintf(sd, "Total data Length: %i", recordLength);
    fclose(sd);
    recording = false;
    Bongo.Lift.stopAll();
    Bongo.Movement.stopAll();
    Bongo.Pneumatics.setRingles(false);
    delay(1000000000);
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

void setPIDSForSubs(double dataToBeReplayed[MaxRecords], double futureDataToBeReplayed[MaxRecords]){
    Bongo.Lift.setPID(dataToBeReplayed, futureDataToBeReplayed);

}

bool needclawToggled = true;
bool needtiltToggled = true;
bool needbackToggled = true;


void controllerVals(double prevDataToBeReplayed[MaxRecords], double dataToBeReplayed[MaxRecords], double futureDataToBeReplayed[MaxRecords], bool useDrive, bool usePos){
    //DRIVE CODE
    if(useDrive){
        Bongo.Movement.tylerControl(dataToBeReplayed[LXDataNum],dataToBeReplayed[LYDataNum],dataToBeReplayed[RXDataNum]);
        Bongo.Movement.move();
    }/* else {
        FL.move_voltage(dataToBeReplayed[FLVolt]);
        FR.move_voltage(dataToBeReplayed[FRVolt]);
        BL.move_voltage(dataToBeReplayed[BLVolt]);
        BR.move_voltage(dataToBeReplayed[BRVolt]);
    }*/
    if(!usePos){
        //manual powering 
        if ((int)dataToBeReplayed[L2ButtonDigital]){
            Bongo.Lift.liftDown(); //manually pushes arm down at max torque
        } else if ((int)dataToBeReplayed[L1ButtonDigital]){
            Bongo.Lift.liftUp(); //manually lifts arm with max torque
        } else {
            Bongo.Lift.stopArm(); //stops manual controll
        }

        if ((int)dataToBeReplayed[R2ButtonDigital]){
            Bongo.Lift.clawDown(); //manually pushes claw down at max torque
            //skillsLift.setAutoLevel(false);
        } else if ((int)dataToBeReplayed[R1ButtonDigital]){
            Bongo.Lift.clawUp(); //manually lifts claw with max torque
            //skillsLift.setAutoLevel(false);
        } else {
            Bongo.Lift.stopClaw(); //stops manual controll
        }
    } else {
        setPIDSForSubs(dataToBeReplayed, futureDataToBeReplayed);
    }
    /*
    printf("Clawe val: %i\n", (int)dataToBeReplayed[AButtonPress]);
    if ((int)dataToBeReplayed[AButtonPress] == 1){
        printf("Triggered claw\n");
        Bongo.Pneumatics.toggleClaw();
    }
    if ((int)dataToBeReplayed[XButtonPress] == 1){
        Bongo.Pneumatics.toggletilt();
    }
    if ((int)dataToBeReplayed[YButtonPress] == 1){
        Bongo.Pneumatics.toggleBack();
    }
    if ((int)dataToBeReplayed[downArrowPress] == 1){
        RingleLift.move_relative(-360, 50);
    }
    if((int)dataToBeReplayed[upArrowPress] == 1){
        Bongo.Pneumatics.toggleRingles();
    }
    */
    
    if ((int)dataToBeReplayed[AButtonDigital] == 1 && (int)prevDataToBeReplayed[AButtonDigital] == 0){
        //printf("Triggered claw\n");
        Bongo.Pneumatics.toggleClaw();
    }
    if ((int)dataToBeReplayed[XButtonDigital] == 1 && (int)prevDataToBeReplayed[XButtonDigital] == 0){
        Bongo.Pneumatics.toggletilt();
    }
    if ((int)dataToBeReplayed[YButtonDigital] == 1 && (int)prevDataToBeReplayed[YButtonDigital] == 0){
        Bongo.Pneumatics.toggleBack();
    }
    if ((int)dataToBeReplayed[downArrowDigital] == 1){
        //RingleLift.move_relative(-360, 50);
    }
    if((int)dataToBeReplayed[upArrowDigital] == 1){
        //Bongo.Pneumatics.toggleRingles();
    }
    
   
   /*
   if ((int)dataToBeReplayed[AButtonDigital] == 1){
        printf("Triggered claw\n");
        Bongo.Pneumatics.clawGrab();
    }
    if ((int)dataToBeReplayed[XButtonDigital] == 1){
        Bongo.Pneumatics.tiltGrab();
    }
    if ((int)dataToBeReplayed[YButtonDigital] == 1){
        Bongo.Pneumatics.backGrab();
    }
    */

}

void checkLeftError(double setPoint){
    leftErrorScore += setPoint - leftOdom.get();
}

void checkRightError(double setPoint){
    rightErrorScore += setPoint + rightOdom.get();
}

double maxVoltage(double checkVolt){
    if(checkVolt > 12000){
        return 12000;
    } else if (checkVolt < 12000){
        return -12000;
    }
    return checkVolt;
}

void encoderVals(double prevDataToBeReplayed[MaxRecords], double dataToBeReplayed[MaxRecords], double futureDataToBeReplayed[MaxRecords]){
    double pVal = 50;
    double dVal = 0;
    double v_Pval = .5; // to input what the motor should be around

    double headingPval = 350;

    //errors
    double leftError = futureDataToBeReplayed[leftOdomPosition] - leftOdom.get();
    double rightError = futureDataToBeReplayed[rightOdomPosition] - rightOdom.get();

    double leftactV = dataToBeReplayed[FLVolt] + dataToBeReplayed[BLVolt];
    double rightactV = dataToBeReplayed[FRVolt] + dataToBeReplayed[BRVolt];

    double headingError = futureDataToBeReplayed[VincentRotation] - Vincent.get_rotation();
    
    double leftSpeed = leftError * pVal + (leftError - lastLeftError) * dVal + leftactV * v_Pval + headingError * headingPval;
    double rightSpeed = rightError * pVal + (rightError - lastRightError) * dVal + rightactV * v_Pval - headingError * headingPval;

    lastLeftError = leftError;
    lastRightError = rightError;


    Bongo.Movement.moveLeftVolt(leftSpeed);
    Bongo.Movement.moveRightVolt(rightSpeed);

    //printf("leftSpeed: %f\n", leftSpeed);
    //printf("rightSpeed: %f\n", rightSpeed);


    controllerVals(prevDataToBeReplayed, dataToBeReplayed, futureDataToBeReplayed, false, true);

    checkLeftError(dataToBeReplayed[leftOdomPosition]);
    checkRightError(dataToBeReplayed[rightOdomPosition]);

}

//returns last index, runs motor values 
void runSparseSegment(double dataToBeReplayed[][3], int dataLength, int timeToRun){
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
    //controllerVals(dataLine, true); fix with encoders
}

//returns last index, runs motor values 
void runSegment(double dataToBeReplayed[][MaxRecords], int index){
    //run off of controller values
    //controllerVals(dataToBeReplayed, false);
    //run off of encoderPID
    encoderVals(dataToBeReplayed[index - 1], dataToBeReplayed[index], dataToBeReplayed[index+1]);
}

void executeData(double dataToBeReplayed[][MaxRecords], int dataLength, int dataTime){
    printf("Executing Data\n");
    dataTime = (dataTime * 1000) / driverSpeed;
    printf("Total Data time: %f\n", (double)dataTime);
    for(int i = 0; i < dataTime; i++){
        //printf("running line: %f\n", (double)i);
        //printf("total: %f\n", (double)dataLength);
        runSegment(dataToBeReplayed, i); //similate inputs 
        delay(driverSpeed); // NEEDS to be the same as driver collected dataLine
    }
    printf("Total leftError: %f\n", leftErrorScore);
    printf("Total rightError: %f\n", rightErrorScore);
    printf("Total LarmError: %f\n", Bongo.Lift.LarmError);
    printf("Total RarmError: %f\n", Bongo.Lift.RarmError);
    printf("Total ClawError: %f\n", Bongo.Lift.ClawError);


}

void resetErrors(){
    printf("Reseting all values\n");
    leftErrorScore = 0;
    rightErrorScore = 0;
    FL.tare_position();
    FR.tare_position();
    BL.tare_position();
    BR.tare_position();
    Larm.tare_position();
    Rarm.tare_position();
    Claw.tare_position();
    leftOdom.reset();
    rightOdom.reset();
    Vincent.tare_rotation();
    Wrist.tare_rotation();


}

void learnEncoder(double dataToBeReplayed[][MaxRecords], int dataLength, int dataTime){
    double incriment  = .5;
    bool increase = true;
    double lastLeftError = 0;
    double lastRightError = 0;
    while(true){
        executeData(dataToBeReplayed, dataLength, dataTime);
        double totalError = leftErrorScore + rightErrorScore;
        double lastTotalError = lastLeftError + lastRightError;
        if(totalError > lastTotalError){
            printf("Higher Error, switching increase/decrease \n");
            increase = !increase;
        } else if (totalError < lastTotalError){
            printf("Lower Error\n");
        } else {
            printf("Somehow it was perfect\n");
            printf("Pval: %f", learnPval);
            break;
        }
        learnPval += incriment * (increase?1:-1);
        printf("New Pval: %f\n", learnPval);
        lastLeftError = leftErrorScore;
        lastRightError = rightErrorScore;
        resetErrors();
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
