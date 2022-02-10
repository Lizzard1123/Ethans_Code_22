#ifndef AUTON
#define AUTON
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
#define LXDataNum 0
#define LYDataNum 1
#define RXDataNum 2
#define RYDataNum 3

#define upArrowDigital 4
#define rightArrowDigital 5
#define downArrowDigital 6
#define leftArrowDigital 7

#define upArrowPress 8
#define rightArrowPress 9
#define downArrowPress 10
#define leftArrowPress 11

#define XButtonDigital 12
#define AButtonDigital 13
#define BButtonDigital 14
#define YButtonDigital 15

#define XButtonPress 16
#define AButtonPress 17
#define BButtonPress 18
#define YButtonPress 19

#define L1ButtonDigital 20
#define L2ButtonDigital 21
#define R1ButtonDigital 22
#define R2ButtonDigital 23

#define L1ButtonPress 24
#define L2ButtonPress 25
#define R1ButtonPress 26
#define R2ButtonPress 27

// sensors
/*
extern Motor FL;
extern Motor FR;
extern Motor BL;
extern Motor BR;
extern Motor Claw;
extern Motor RingleLift;
extern Motor Larm;
extern Motor Rarm;
extern okapi::ADIEncoder  rightOdom;
extern pros::c::ext_adi_encoder_t  leftOdom;
*/

#define FLActualVelocity 28
#define FRActualVelocity 29
#define BLActualVelocity 30
#define BRActualVelocity 31
#define ClawActualVelocity 32
#define RingleLiftActualVelocity 33
#define LarmActualVelocity 34
#define RarmActualVelocity 35

#define ClawPosition 36
#define RingleLiftPosition 37
#define LarmPosition 38
#define RarmPosition 39
#define rightOdomPosition 40
#define leftOdomPosition 41


#define FLVolt 42
#define FRVolt 43
#define BLVolt 44
#define BRVolt 45
#define LarmVolt 46
#define RarmVolt 47
#define ClawVolt 48

#define VincentRotation 49
#define WristRotation 50
#define VincentPitch 51

#define recordedX 52
#define recordedY 53

#define MaxRecords 54


extern void fillEmpty();

extern void setData();

extern void finalizeData();

extern void printUnfilteredData();

extern void printData();

extern void printDataToSD();

extern void runSegment(double dataToBeReplayed[][MaxRecords], int timeToRun);

extern void executeData(double dataToBeReplayed[][MaxRecords], int dataLength, int dataTime);

extern bool isRecording();

extern void setRecording(bool val);

extern void startRecording();

extern void stopRecording();

extern bool hasRecorded();

extern void setDataToSd();

extern void learnEncoder(double dataToBeReplayed[][MaxRecords], int dataLength, int dataTime);

#define skills_dataLength 300

#define auton_dataLength 300

extern int skillsDataTime;

extern int autonDataTime;

extern double skills_Data[][MaxRecords];

extern double auton_Data_RRS[][MaxRecords];

extern double auton_Data_RRM[][MaxRecords];

extern double auton_Data_HR[][MaxRecords];

extern double auton_Data_RLS[][MaxRecords];



#endif