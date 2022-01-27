#ifndef GLOBAL
#define GLOBAL
#include "main.h"
#define driverSpeed 50 //in ms
//motor ports
#define FLPort 20
#define FRPort 13
#define BLPort 19
#define BRPort 11
#define LarmPort 18
#define RarmPort 16
#define ClawPort 12
#define LiftPort 17
//#define EYESPort 4
//other defines
#define expanderPort 14


//motors
extern Motor FL;
extern Motor FR;
extern Motor BL;
extern Motor BR;
extern Motor Claw;
extern Motor RingleLift;
extern Motor Larm;
extern Motor Rarm;

//controllers
extern Controller master;
extern Controller partner;

//vision sensors
//extern Vision Eyes;
//extern Vision leftEye;
//extern Vision rightEye;
//extern Vision backEye;

//sigs
//#define EYES__CUSTOM_GREEN_NUM 1
//#define MOGO_CUSTOM_YELLOW_NUM 1
//extern vision_signature_s_t  EYES__CUSTOM_GREEN;
//extern vision_signature_s_t MOGO_CUSTOM_YELLOW;

//opical sensor
//extern Optical Police;

//pots
//extern ADIAnalogIn liftPot;
//extern ADIAnalogIn leftWingPot;
//extern ADIAnalogIn rightWingPot;

//LED
//extern ADIDigitalOut led;
extern ADIDigitalOut tiltLock;
extern ADIDigitalOut backLock;
extern ADIDigitalOut liftShifter;
extern ADIDigitalOut clawLock;


//limit switch / button
extern ADIDigitalIn leftSwitch;


//Odom
extern okapi::ADIEncoder  rightOdom;
extern okapi::ADIEncoder  leftOdom;
//extern okapi::ADIEncoder  middleOdom;

//pots
//extern ADIAnalogIn leftArmPot;
//extern ADIAnalogIn rightArmPot;

//labels debug
extern lv_obj_t *debugXLabel;
extern lv_obj_t *debugYLabel;
extern lv_obj_t *debugRotationLabel;
extern lv_obj_t *debugXarmLabel;
extern lv_obj_t *debugYarmLabel;

//global teamcolor
extern bool teamIsBlue;
extern double maxSpeedMultiplier;
#endif
