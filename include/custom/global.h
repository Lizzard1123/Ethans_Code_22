#ifndef GLOBAL
#define GLOBAL
#include "main.h"
//motor ports
#define FLPort 1 
#define FRPort 20
#define BLPort 2
#define BRPort 8 
#define LarmPort 3
#define RarmPort 9
#define ClawPort 7
#define LiftPort 15
//other defines
#define LeftSideNum 0
#define RightSideNum 1
#define BackSideNum 2
#define expanderPort 10


//motors
extern Motor FL;
extern Motor FR;
extern Motor BL;
extern Motor BR;
extern Motor Claw;
extern Motor Lift;
extern Motor Larm;
extern Motor Rarm;

//controllers
extern Controller master;
extern Controller partner;

//vision sensors
//extern Vision EYES;
//extern Vision leftEye;
//extern Vision rightEye;
//extern Vision backEye;

//sigs
//#define EYES__CUSTOM_GREEN_NUM 1
#define MOGO_CUSTOM_YELLOW_NUM 1
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
extern ADIDigitalOut frontLock;
extern ADIDigitalOut backLock;
extern ADIDigitalOut liftShifter;
extern ADIAnalogOut clawLock;


//limit switch / button
//extern ADIDigitalIn tailSensor;

//Odom
extern okapi::ADIEncoder  leftOdom;
extern pros::c::ext_adi_encoder_t  rightOdom;
//extern okapi::ADIEncoder  middleOdom;

//pots
//extern ADIAnalogIn TeamColor;

//labels debug
extern lv_obj_t *debugXLabel;
extern lv_obj_t *debugYLabel;
extern lv_obj_t *debugRotationLabel;
extern lv_obj_t *debugXarmLabel;
extern lv_obj_t *debugYarmLabel;

//global teamcolor
extern bool teamIsBlue;

#endif

