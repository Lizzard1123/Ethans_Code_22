#ifndef ROBOT
#define ROBOT
#include "lift.h"
#include "movement.h"
#include "pneumatics.h"
#include <climits>

class Robot
{
private:
    // global X pos of bongo declared in global.cpp
    static double X;
    // global Y pos of bongo declared in global.cpp
    static double Y;
    //current heading of bongo
    static double rotation;
    // global position update delay for bongo declared in global.cpp
    static const double posDelay;
    // global team color of bongo declared in global.cpp
    static bool teamIsBlue;
    // auton selector num
    int autonCodeNum;
    // true if bongo is on left for auton
    bool left;
    // bongo has been initialized
    bool initz = false;
    

    //tower PID vars
    // neg right pos left angle
    double mt_offset = 158;
    double mt_error;
    double mt_targetCX;
    double mt_CX;
    double mt_width;
    double mt_error_past = 0;
    double mt_Dval = 0;
    double mt_Pval = .2;
    double mt_widthLimit = 20;

    double t_tolerance = .5;
    double t_Pval = 2; //2.5
    double t_Ival = 0;//double Ival = .1;// double Ival = 0;
    double t_Dval = 1;//double Dval = .7;// double Dval = 0;

    int PIDspeed = 50;// in ms
    double m_tolerance = .2;// tolerance in inches
    double m_Pval = 8; //13
    double m_Ival = .25; //0
    double m_Dval = 0; //0

    static double totalForwardMovement;

public:
    static LiftClass Lift;
    // class handler for movement + other funtions
    static RobotMovement Movement;
    // custom math reference
    static Math myMath;
    //custom static pneumatics class
    static PneumaticsClass Pneumatics;

    // sets team color to red
    void setRed()
    {
        teamIsBlue = false;
    }

    // sets team color to blue
    void setBlue()
    {
        teamIsBlue = true;
    }

    // sets auton number
    void setAutonNum(int num)
    {
        autonCodeNum = num;
    }

    // sets auton side, true is left
    void sideIsLeft(bool yes)
    {
        left = yes;
    }

    // returns true if team is blue
    bool getColor()
    {
        return teamIsBlue;
    }

    // returns auton num
    int getAutonNum()
    {
        return autonCodeNum;
    }

    //returns x variable
    double getX()
    {
        return X;
    }
    //returns y variable
    double getY()
    {
        return Y;
    }
    //returns the rotation of bongo
    static double getRotation(){
        return rotation;
    }
    // returns if bongo is on left during auton
    bool getSide()
    {
        return left;
    }

    void autonomous(){
        Autonomous(autonCodeNum = 1, left = false, teamIsBlue = true);
    }

    void catieControl(){
        Movement.catieControl(rotation);
    }

    void tylerControl(){
        Movement.tylerControl();
    }

    // PID syncronous movement from current location to target X , Y set speed along the way
    int PIDMove(double targetX, double targetY, double maxspeed = 100)
    {
        bool reachedGoal;
        double speed = 0;
        double prevError;
        double error;
        double derivative;
        double integral = 0;

        while (true)
        {
            // find the error distance bertween current and target point
            error = myMath.TwoPointsDistance(X, Y, targetX, targetY);

            // find the intergral part for I
            if ((speed != 0) && (speed <= 15) && (integral < 1000) && (error < 1000))
            {
                integral += error;
            }

            // find the derivative part for D
            //this should be change of error 
            derivative =  error / PIDspeed;

            // PID ALGO
            speed = (error * m_Pval) + (integral * m_Ival) + (derivative * m_Dval);

            // get current angle
            double Dangle = myMath.angleBetween(X, Y, targetX, targetY);

            // fancy algo
            double FLAuton = myMath.sRound(
                myMath.multiplier(FLnum, rotation, Dangle) * speed, 3);
            double FRAuton = myMath.sRound(
                myMath.multiplier(FRnum, rotation, Dangle) * speed, 3);
            double BLAuton = myMath.sRound(
                myMath.multiplier(BLnum, rotation, Dangle) * speed, 3);
            double BRAuton = myMath.sRound(
                myMath.multiplier(BRnum, rotation, Dangle) * speed, 3);
            double under = myMath.greatest(fabs(FLAuton), fabs(FRAuton),
                                           fabs(BLAuton), fabs(BRAuton)) /
                           100;
            FLAuton = (FLAuton / under);
            FRAuton = (FRAuton / under);
            BLAuton = (BLAuton / under);
            BRAuton = (BRAuton / under);
            //printf("FLAuton %f \n", FLAuton);
            //printf("FRAuton %f \n", FRAuton);
            //printf("BLAuton %f \n", BLAuton);
            //printf("BRAuton %f  \n", BRAuton);
            //printf("rotation %f  \n", rotation);
            //printf("Dangle %f \n", Dangle);
            //printf("error %f \n", error);
            //printf("X:  %f \n", X);
            //printf("Y:  %f \n", Y);
            // Drive Bongo
            Movement.moveFL(myMath.maxSpeed(FLAuton, maxspeed));
            Movement.moveFR(myMath.maxSpeed(FRAuton, maxspeed));
            Movement.moveBL(myMath.maxSpeed(BLAuton, maxspeed));
            Movement.moveBR(myMath.maxSpeed(BRAuton, maxspeed));

            // if the pid loop has reached target
            if (fabs(error) <= m_tolerance)
            {
                reachedGoal = true;
            }
            else
            {
                reachedGoal = false;
            }

            if (reachedGoal)
            {
                Movement.moveLeft(0);
                Movement.moveRight(0);
                break;
            }

            delay(PIDspeed);
        }
        return 1;
    }

    void PIDStraight(double distance, double keepAngle){
        //others
        double M_Pval = 4;
        double M_Dval = 2.5;
        double T_Pval = 0; //doesnt work
         //pid vars
        double lastError = 0;
        totalForwardMovement = 0;
        while(true){

            double M_error = distance - totalForwardMovement;
            double T_error = keepAngle - rotation;
            double M_derivative = (M_error - lastError)/ PIDspeed;

            double leftSpeed = M_error * M_Pval + M_derivative * M_Dval - T_Pval * T_error;
            double rightSpeed = M_error * M_Pval + M_derivative * M_Dval + T_Pval * T_error;

            Movement.moveLeft(leftSpeed);
            Movement.moveRight(rightSpeed);
            printf("leftSpeed: %f", leftSpeed);
            printf("rightSpeed: %f", rightSpeed);
            printf("M_error: %f", M_error);
            printf("T_error: %f", T_error);

            if(fabs(M_error) < m_tolerance){
                Movement.moveLeft(0);
                Movement.moveRight(0);
                break;
            }

            delay(PIDspeed);
        }

    }


    // PID syncronous turning TODO merge with movement
    int PIDTurn(double target, double maxspeed = 100)
    {
        double motorSpeed = 0;
        double turnTarget = target;
        bool reachedGoal;
        double error;
        double Derivative = 0;
        double integralone = 0;
        double lastError = 0; // should start off being current error not doing that though

        while (true)
        {
            //printf("Heading: %f", headingVal);
            // find the error of both sides  for P
            error = turnTarget - rotation;
            printf("Error: %f \n", error);
            printf("rotation: %f \n", rotation);


            // find the intergral part for I
            // integralone = 0;
            //if ((integralone == -4294967295.4294967295) || (integralone == 1045))
            //{
            //  integralone = 0;
            //}

            //if (motorSpeed <= 20)
            //{
            //  integralone += error;
            //}

            // find the derivative part for D
            Derivative = (error - lastError) / PIDspeed;
            lastError = error;

            // PID ALGO
            motorSpeed = (error * t_Pval) + (integralone * t_Ival) + (Derivative * t_Dval);

            // Drive Bongo
            Movement.moveLeft(myMath.maxSpeed(motorSpeed, maxspeed));
            Movement.moveRight(myMath.maxSpeed(-motorSpeed, maxspeed));

            // if the pid loop has reached target
            if (fabs(error) <= t_tolerance)
            {
                reachedGoal = true;
            }
            else
            {
                reachedGoal = false;
            }

            if (reachedGoal)
            {
                Movement.moveLeft(0);
                Movement.moveRight(0);
                break;
            }

            delay(PIDspeed);
        }
        return 1;
    }

    //PID turnMove turns and moves
    int PIDMoveTurn(double targetX, double targetY, double target, double maxspeed = 100)
    {
        //move PID info
        bool m_reachedGoal;
        double m_speed = 0;
        double m_prevError;
        double m_error;
        double m_derivative;
        double m_integral = 0;

        //turn PID info
        double t_speed = 0;
        bool t_reachedGoal;
        double t_error;
        double t_derivative = 0;
        double t_integral = 0;

        while (true)
        {
            // find the error distance bertween current and target point
            m_error = myMath.TwoPointsDistance(X, Y, targetX, targetY);
            // find the error of both sides for P
            t_error = target - rotation;

            // find the intergral part for I
            if ((m_speed != 0) && (m_speed <= 15) && (m_integral < 1000) && (m_error < 1000))
            {
                m_integral += m_error;
            }

            // find the derivative part for D
            //this should be change of error 
            m_derivative =  m_error / PIDspeed;

            // PID ALGO
            m_speed = (m_error * m_Pval) + (m_integral * m_Ival) + (m_derivative * m_Dval);
            // PID ALGO
            t_speed = (t_error * t_Pval) + (t_integral * t_Ival) + (t_derivative * t_Dval);

            // get current angle
            double Dangle = myMath.angleBetween(X, Y, targetX, targetY);

            // fancy algo
            double FLAuton = myMath.sRound(
                myMath.multiplier(FLnum, rotation, Dangle) * m_speed + t_speed, 3);
            double FRAuton = myMath.sRound(
                myMath.multiplier(FRnum, rotation, Dangle) * m_speed - t_speed, 3);
            double BLAuton = myMath.sRound(
                myMath.multiplier(BLnum, rotation, Dangle) * m_speed + t_speed, 3);
            double BRAuton = myMath.sRound(
                myMath.multiplier(BRnum, rotation, Dangle) * m_speed - t_speed, 3);
            double under = myMath.greatest(fabs(FLAuton + t_speed), fabs(FRAuton + t_speed),
                                           fabs(BLAuton + t_speed), fabs(BRAuton + t_speed)) /
                           100;
            FLAuton = (FLAuton / under);
            FRAuton = (FRAuton / under);
            BLAuton = (BLAuton / under);
            BRAuton = (BRAuton / under);
            //printf("FLAuton %f \n", FLAuton);
            //printf("FRAuton %f \n", FRAuton);
            //printf("BLAuton %f \n", BLAuton);
            //printf("BRAuton %f  \n", BRAuton);
            printf("Heading: %f", rotation);
            printf("Dangle %f \n", Dangle);
            printf("target %f \n", target);
            printf("error %f \n", m_error);
            printf("error %f \n", t_error);
            printf("X:  %f \n", X);
            printf("Y:  %f \n", Y);
            // Drive Bongo
            Movement.moveFL(myMath.maxSpeed(FLAuton, maxspeed));
            Movement.moveFR(myMath.maxSpeed(FRAuton, maxspeed));
            Movement.moveBL(myMath.maxSpeed(BLAuton, maxspeed));
            Movement.moveBR(myMath.maxSpeed(BRAuton, maxspeed));

            // if the pid loop has reached target
            if (fabs(m_error) <= m_tolerance)
            {
                m_reachedGoal = true;
            }
            else
            {
                m_reachedGoal = false;
            }

            if (fabs(t_error) <= t_tolerance)
            {
                t_reachedGoal = true;
            }
            else
            {
                t_reachedGoal = false;
            }

            if (m_reachedGoal && t_reachedGoal)
            {
                Movement.moveLeft(0);
                Movement.moveRight(0);
                break;
            }
            //delay
            delay(PIDspeed);
        }
        return 1;
    }

    void resetOdom()
    {
        X = 0;
        Y = 0;
        rotation = 0;
    }

    static void updatePos(void *)
    {
        //I derived the original formula and for the reiteration and added wheel i combined it with work done here
        //https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf
        double wheelCircumfrence = 11.2; //10.19
        double wheelSmallCircumfrence = 8.64;
        //bigger increases angle more
        double wheelSeperation = 6.45 * 2; //4.4
        double wheelOffset = 5.5;
        double head = rotation;
        double rightOdomVal;
        double leftOdomVal;
        double middleOdomVal;
        while (true)
        {
            //just in case i forget to code properly
            if (std::isnan(X) || fabs(X) > 144)
            {
                X = 0;
            }
            if (std::isnan(Y) || fabs(Y) > 144)
            {
                Y = 0;
            }
            
            
            //find isolated forward and sideways movement
            double leftDist = myMath.toInch(pros::c::ext_adi_encoder_get(leftOdom), wheelCircumfrence);
            double rightDist = myMath.toInch(rightOdom.get(), wheelCircumfrence);
            double forwardMovement = (rightDist + leftDist) / 2;
            totalForwardMovement += forwardMovement;
            double sidewaysMovement = 0;// myMath.toInch(middleOdom.get(), wheelSmallCircumfrence);
            double changeOfHeading = ((leftDist - rightDist) / wheelSeperation) * 180 / M_PI;
            //to distance
            //sidewaysMovement += wheelOffset * (changeOfHeading  * M_PI / 180);

            // forward wheels in relation to coordinates
            //HEY YOU
            //mit student says switch cos and sin on these here if your code breaks its bc hes smarter than you and you need to fix this
            Y += forwardMovement * cos(head * M_PI / 180);
            X += forwardMovement * sin(head * M_PI / 180);
            //crackhead
            // sideways wheel in relation to cooridantes
            //Y -= sidewaysMovement * sin(head * M_PI / 180);
            //X += sidewaysMovement * cos(head * M_PI / 180);
            //heading//update heading part
            
            rotation += changeOfHeading;
            //set new
            //TODO i thought this went before but try accuracy when its after computed distance moved idk i think its right
            head = rotation;// TODO fix heading
            //debug
            //printf("right: %f\n", rightDist);
            //printf("left: %f\n", leftDist);
            //printf("heading %f\n", head);
            //printf("Rotation %f\n", rotation);
            //printf("Current X %f\n", X);
            //printf("Current Y %f\n", Y);
            //printf("forward movement %f\n", forwardMovement);
            //printf("\n");
            //printf("heading change %f\n", changeOfHeading);
            //printf("sideways movement %f \n", sidewaysMovement);
            //printf("Back wheel rotation %f\n", myMath.toInch(sidewaysMovement, wheelSmallCircumfrence));
            //printf("\n");
            // reset
            rightOdom.reset();
            pros::c::ext_adi_encoder_reset(leftOdom);
            //middleOdom.reset();
            c::task_delay(posDelay);
        }
    }

    void testOdom()
    {
        //PIDTurn(180);
        turnTimed(.6, false, 50); //right
    }

    void testOdom3(){
        PIDTurn(360);
        //PIDMoveTurn(0, 72, 0, 100);
    }

    void testOdom2()
    {
        PIDTurn(90);
        //PIDMoveTurn(72, 0, 0, 100);
    }

    // set current position of bongo
    void setPos(double x, double y)
    {
        X = x;
        Y = y;
    }

    void setRotation(double r){
        rotation = r;
    }

    // prints to debug screen current position of bongo
    void debugPos()
    {
        std::string xPos = "X: " + std::to_string(X);
        std::string yPos = "Y: " + std::to_string(Y);
        std::string roationPos = "roation: " + std::to_string(rotation);
        lv_label_set_text(debugXLabel, xPos.c_str());
        lv_label_set_text(debugYLabel, yPos.c_str());
        lv_label_set_text(debugRotationLabel, roationPos.c_str());
    }

    // change current team (swap) pressing L1 and L2 at same time when called
    void changeTeam()
    {
        if ((master.get_digital(E_CONTROLLER_DIGITAL_L1)) &&
            (master.get_digital(E_CONTROLLER_DIGITAL_L2)))
        {
            teamIsBlue = !teamIsBlue;
        }
    }

    //returns true if bongo has initialized
    bool isinit()
    {
        return initz;
    }

    //calls upon subsystems and updates them
    static void updateLiftSpeed(void *)
    {
        while (true)
        {
            //update subsystem motors in their methods respectivley 
            Lift.update();
            c::task_delay(10);
        }
    }

    //starts up threads
    void initThreads()
    {//updateLift
        Task updateLift(updateLiftSpeed, nullptr, TASK_PRIORITY_DEFAULT,
                            TASK_STACK_DEPTH_DEFAULT, "updateLift");
        // track locationFty
        Task updatePosition(updatePos, nullptr, TASK_PRIORITY_DEFAULT,
                            TASK_STACK_DEPTH_DEFAULT, "updatePos");
        
        initz = true;
    }

    //async PID for changing angle to mogo
    /*
    double lineUpMogo(int sideNum, bool moveThem = false)
    {
        // take signature from side
        vision_object_s_t mogo;
        //objects on right will require motors to move in opposite direction of left and back
        int rotationMultiplier;
        switch (sideNum)
        { //TODO fix this rotation multiplier idk which is which
        case LeftSideNum:
            mogo = leftEye.get_by_sig(0, MOGO_CUSTOM_YELLOW_NUM);
            rotationMultiplier = 1;
            break;
        case RightSideNum:
            mogo = rightEye.get_by_sig(0, MOGO_CUSTOM_YELLOW_NUM);
            rotationMultiplier = -1;
            break;
        case BackSideNum:
            mogo = backEye.get_by_sig(0, MOGO_CUSTOM_YELLOW_NUM);
            rotationMultiplier = 1;
            break;
        }
        if (mogo.width >= mt_widthLimit)
        {
            mt_width = mogo.width;
            mt_CX = mogo.x_middle_coord;
            //custom formula for dist turn
            printf("Error: %f", mt_error);
            //158 is center of vison view
            mt_targetCX = 158 - mt_CX + mt_offset;
            mt_error = mt_CX - mt_targetCX;
            // RN its just a P loop D if you add this + Dval * ((error - error_past) / 5) time delay
            Movement.addToFLspeed(mt_Pval * mt_error * rotationMultiplier);
            Movement.addToFRspeed(mt_Pval * -mt_error * rotationMultiplier);
            Movement.addToBLspeed(mt_Pval * mt_error * rotationMultiplier);
            Movement.addToBRspeed(mt_Pval * -mt_error * rotationMultiplier);
            if (moveThem)
            {
                Movement.move();
            }
            mt_error_past = mt_error;
        }
        return mt_error;
    }
    */
    //AUTONS
    void moveForwardTimedLineUp(double time, double speed = 100){
        double interval = time / 10; //10 ms 
        int count = 0;
        double turnPval = 1;
        while(count < interval){
            vision_object_s_t mogo = Eyes.get_by_sig(0, MOGO_CUSTOM_YELLOW_NUM);
            double turnVal = (158 - mogo.x_middle_coord) * turnPval;
            Movement.moveLeft(speed + turnVal);
            Movement.moveRight(speed - turnVal);
        }
        
        Movement.stopAll();
    }

    void moveForwardTimed(double time, double speed = 100){
        Movement.moveLeft(speed);
        Movement.moveRight(speed);
        delay(time*1000);
        Movement.stopAll();
    }

    void turnTimed(double time, bool left, double speed = 100){
        Movement.moveLeft(-speed * (left ? -1 : 1));
        Movement.moveRight(speed * (left ? -1 : 1));
        delay(time*1000);
        Movement.stopAll();
    }

    void moveBackwardTimed(double time, double speed = 100){
        Movement.moveLeft(-speed);
        Movement.moveRight(-speed);
        delay(time*1000);
        Movement.stopAll();
    }

    void strafeTimed(double time, bool reverse, double speed = 100){
        Movement.moveFL(-speed * (reverse ? -1 : 1));
        Movement.moveFR(speed * (reverse ? -1 : 1));
        Movement.moveBL(speed * (reverse ? -1 : 1));
        Movement.moveBR(-speed * (reverse ? -1 : 1));
        delay(time*1000);
        Movement.stopAll();
    }

    void AutonomousOne(bool isLeft, bool isBlue){
        if(isLeft){
            setRotation(25);
            Pneumatics.clawRelease();
            moveForwardTimed(.25);
            moveBackwardTimed(.25);
            Pneumatics.clawRelease();
            //moveForwardTimedLineUp(1.4);
            moveForwardTimed(1.4);
            Pneumatics.clawGrab();
            //drop turn lift manuver
            totalForwardMovement = 0;
            moveBackwardTimed(1.25);
            //check for tugging
            while(totalForwardMovement > -20){
                moveBackwardTimed(.25);
            }
            Pneumatics.clawRelease();
            moveBackwardTimed(.15);
            //sus code
            Lift.liftUp();
            PIDTurn(rotation - 100);
            //strafe left to wall
            strafeTimed(1.4, false, 50);
            //right
            strafeTimed(.4, true, 50);
            Lift.stopAll();
            //rotate and grab
            Pneumatics.toggletilt();
            Pneumatics.toggleRingles();
            moveBackwardTimed(.6, 50);
            Pneumatics.toggleBack();
            Pneumatics.toggletilt();
            moveForwardTimed(2, 30);
            moveBackwardTimed(2, 30);
            moveForwardTimed(2, 30);
            moveBackwardTimed(2, 30);
        } else { //right small
            setRotation(0);
            Pneumatics.clawRelease();
            Lift.liftUp();
            Lift.clawDown();
            delay(300);
            Lift.liftDown();
            Lift.clawUp();
            moveForwardTimed(.3);
            Lift.stopAll();
            //moveForwardTimedLineUp(1.4);
            moveForwardTimed(.8);
            Pneumatics.clawGrab();

            totalForwardMovement = 0;
            moveBackwardTimed(.6);

            //check for tugging
            while(totalForwardMovement > -12){
                moveBackwardTimed(.25);
            }
            
            delay(1000000); //check rotation

            //turn and drop
            //PIDTurn(-90);
            turnTimed(turnTime,  false, 50); //turn right
            Pneumatics.clawRelease();
            Lift.liftUp();

            //rotate and grab
            Pneumatics.toggletilt();
            Pneumatics.toggleRingles();
            moveBackwardTimed(1, 50);
            Lift.stopAll();
            Pneumatics.toggleBack();
            Pneumatics.toggletilt();
            
            //move to line up 
            //PIDTurn(0);
            turnTimed(turnTime,  true, 50); //turn left
            strafeTimed(1.4, true, 100); //right
            strafeTimed(1, false, 50); //left

            //lined up with the ringles
            moveForwardTimed(2, 30);

            //point turn to get to home zone and slow for more rings
            //PIDTurn(180);
            turnTimed(2*turnTime,  false, 50); //turn right

            moveForwardTimed(.5, 100);
            moveForwardTimed(2, 30);
        }
    };
    void AutonomousTwo(bool isLeft, bool isBlue){
        if(isLeft){
            setRotation(40);
        } else {
            setRotation(-40);
        }
        //Pneumatics.toggleClaw();
        //delay(.25);
        Pneumatics.clawRelease();
        moveForwardTimed(.25);
        moveBackwardTimed(.25);
        Pneumatics.clawRelease();
        moveForwardTimed(2.25);
        Pneumatics.clawGrab();
        //tug
        totalForwardMovement = 0;
        moveBackwardTimed(1.25);
        //check for tugging
        while(totalForwardMovement > -20){
            moveBackwardTimed(.25);
        }
    };
    void AutonomousThree(bool isLeft, bool isBlue){
        //TODO finish this auton pathing
    };
    void AutonomousFour(bool isLeft, bool isBlue){
        //TODO finish this auton pathing
    };

    void Autonomous(int num, bool isLeft, bool isBlue)
    {
        switch (num)
        {
        case 1:
            AutonomousOne(isLeft, isBlue);
            break;

        case 2:
            AutonomousTwo(isLeft, isBlue);
            break;

        case 3:
            AutonomousThree(isLeft, isBlue);
            break;
        case 4:
            AutonomousFour(isLeft, isBlue);
            break;
        }
    }
};

extern Robot Bongo;

#endif // ifndef ROBOT