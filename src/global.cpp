#include "custom/robot.h"

// motors
//TODO change the motor cartriages of the motors (36, 18, 6)
//TODO if motor is reversed then this boolean param flips
Motor FL(FLPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor FR(FRPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor BL(BLPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES);
Motor BR(BRPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor Larm(LarmPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES); 
Motor Rarm(RarmPort, E_MOTOR_GEARSET_18, true, E_MOTOR_ENCODER_DEGREES); 
Motor Claw(ClawPort, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES); 
Motor Lift(LiftPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES); 


// controllers
Controller master(E_CONTROLLER_MASTER);
Controller partner(E_CONTROLLER_PARTNER);

// vision sensors
//Vision EYES(EYESPort);
//Vision leftEye(leftEyePort);
//Vision rightEye(rightEyePort);
//Vision backEye(backEyePort);

//signatures for vision sensor 
//vision_signature_s_t EYES__CUSTOM_GREEN = Vision::signature_from_utility(EYES__CUSTOM_GREEN_NUM, -4949, -4509, -4729, -5669, -5035, -5352, 9.400, 0);
//vision_signature_s_t MOGO_CUSTOM_YELLOW = Vision::signature_from_utility(MOGO_CUSTOM_YELLOW_NUM, 189, 1879, 1034, -3857, -2845, -3352, 0.700, 0); //TODO get the value of mogo sig
/*
vision::signature YLW (1, -623, 257, -182, -4361, -3217, -3790, 4.500, 0);
vision::signature SIG_1 (1, 189, 1879, 1034, -3857, -2845, -3352, 0.700, 0);
*/
//Optical sensor
//Optical Police(PolicePort);

//LED/digital out for pnematics
//ADIDigitalOut led('F', 1);
//ADIDigitalOut  leftLock ({{expanderPort, leftLockPort}});
//ADIDigitalOut  rightLock ({{expanderPort, rightLockPort}});
ADIDigitalOut clawLock('E');
ADIDigitalOut tiltLock('G');
ADIDigitalOut backLock('H');



//button / limit switch
//ADIDigitalIn tailSensor(tailSensorPort);

//pots
//ADIAnalogIn liftPot('c');
//ADIAnalogIn leftArmPot('D');
//ADIAnalogIn rightArmPot('F');

// Odom 
okapi::ADIEncoder  rightOdom('A', 'B', true); //true
pros::c::ext_adi_encoder_t leftOdom = pros::c::ext_adi_encoder_init(expanderPort, 'G', 'H', false); //false
//okapi::ADIEncoder  middleOdom('a', 'b', false);

// Staic defines
RobotMovement Robot::Movement;
LiftClass Robot::Lift;
Math Robot::myMath;
PneumaticsClass Robot::Pneumatics;
bool Robot::teamIsBlue = false;
const double Robot::posDelay = 50;
double Robot::X = 0;
double Robot::totalForwardMovement = 0;
double Robot::Y = 0;
double Robot::rotation = 0;
