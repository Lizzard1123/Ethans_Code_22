#ifndef ROBOT
#define ROBOT
#include "lift.h"
#include "movement.h"
#include "pneumatics.h"
#include "auton.h"
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
    double mt_Dval = 1;
    double mt_Pval = .2;
    double mt_widthLimit = 20;

    double t_tolerance = 1;
    double t_Pval = 1.5; //2.5
    double t_Ival = 0;//double Ival = .1;// double Ival = 0;
    double t_Dval = 4;//double Dval = .7;// double Dval = 0;

    int PIDspeed = 50;// in ms
    double m_tolerance = .2;// tolerance in inches
    double m_Pval = 8; //13
    double m_Ival = 0; //0
    double m_Dval = 1; //0

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
        //Autonomous(getAutonNum(), getSide(), getColor());
        Autonomous(4, false, false);
    }

    void catieControl(){
        Movement.catieControl(rotation);
    }

    void tylerControl(){
        Movement.tylerControl();
    }

    void arcadeControl(){
        Movement.arcadeControl();
    }

    void PIDClimb(){
        double k_Pval = 3;
        double k_Dval = 45;
        double error;
        double lastError;
        double target = Vincent.get_pitch();
        //lineup and push down
        delay(1500);
        moveForwardTimed(.5,50);
        delay(600000);
        delay(500);
        while(true){
            error = target - Vincent.get_pitch();
            double speed = error * k_Pval + (error - lastError) * k_Dval;
            Movement.moveLeft(speed);
            Movement.moveRight(speed);
            lastError = error;
            if(error <= 2){
                Movement.moveLeft(0);
                Movement.moveRight(0);
                break;
            }
            delay(10);
        }
    }

    // PID syncronous movement from current location to target X , Y set speed along the way
    int PIDMove(double targetX, double targetY, bool backwards, double maxspeed = 100)
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
            double angle = myMath.angleBetween(X, Y, targetX, targetY);
            if(angle >= fabs(90)){
                backwards =  true;
            }
            error = myMath.TwoPointsDistance(X, Y, targetX, targetY);
            double PIDval = (error * m_Pval + (error - prevError) * m_Dval) * (backwards?-1:1);
            prevError = error;
            double turnCorrection = (Vincent.get_rotation() - angle + (backwards?180:0)) * t_Pval/2;
            Movement.moveFL(myMath.maxSpeed(PIDval + turnCorrection, maxspeed));
            Movement.moveFR(myMath.maxSpeed(PIDval - turnCorrection, maxspeed));
            Movement.moveBL(myMath.maxSpeed(PIDval + turnCorrection, maxspeed));
            Movement.moveBR(myMath.maxSpeed(PIDval - turnCorrection, maxspeed));

            printf("error: %f\n", error);
            printf("PIDval: %f\n", PIDval);
            printf("turnCorrection: %f\n", turnCorrection);
            printf("angle: %f\n", myMath.angleBetween(X, Y, targetX, targetY));
            printf("back?: %i\n", backwards?180:0);

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
            error =  turnTarget - Vincent.get_rotation();
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
            Derivative = (error - lastError);
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
    int PIDMoveTurn(double targetX, double targetY, double target, bool backwards, double maxspeed = 100)
    {
        PIDTurn(myMath.angleBetween(X, Y, targetX, targetY) + backwards?180:0);
        PIDMove(targetX, targetY, backwards);
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
        double wheelCircumfrence = 10.2; //11.2
        double head = rotation;
        double rightOdomVal;
        double leftOdomVal;
        double lastRightOdomVal;
        double lastLeftOdomVal;

        double rightOdomDist;
        double leftOdomDist;
        while (true)
        {
            if (isnan(X))
            {
                X = 0;
            }
            if (isnan(Y))
            {
                Y = 0;
            }
            rightOdomVal = rightOdom.get();
            leftOdomVal = leftOdom.get();
            rightOdomDist = myMath.toInch(rightOdomVal - lastRightOdomVal, wheelCircumfrence);
            leftOdomDist = myMath.toInch(leftOdomVal - lastLeftOdomVal, wheelCircumfrence);
            double changeHeading = head - Vincent.get_rotation();
            double moveDist = leftOdomDist - (leftOdomDist * (changeHeading * M_PI / 180)); //(rightOdomDist + leftOdomDist) / 2;
            totalForwardMovement += rightOdomVal;

            //printf("h %f \n", head);
            //printf("v %f \n", rightOdomVal);
                // YWhee
            Y += moveDist * cos(head * M_PI / 180);
            X += moveDist * sin(head * M_PI / 180);

            //debug
            //printf("X: %f\n", X);
            //printf("Y: %f\n", Y);
            // reset
            //prev = head;
            //rightOdom.reset();
            //leftOdom.reset();
            lastRightOdomVal = rightOdomVal;
            lastLeftOdomVal = leftOdomVal;
            head = Vincent.get_rotation();
            rotation = head;
            std::string values = std::__cxx11::to_string(int(FL.get_temperature())) + ":" +  std::__cxx11::to_string(int(FR.get_temperature())) + ":" +  std::__cxx11::to_string(int(BL.get_temperature())) + ":" +  std::__cxx11::to_string(int(BR.get_temperature())) + ":" +  std::__cxx11::to_string(int(Rarm.get_temperature())) + ":" +  std::__cxx11::to_string(int(Larm.get_temperature())) + ":" + std::__cxx11::to_string(int(Claw.get_temperature()));
            master.set_text(1, 0, values);
            values = std::__cxx11::to_string(int(leftSwitch.get_value())) + ":" +  std::__cxx11::to_string(int(rightOdom.get())) + ":" +  std::__cxx11::to_string(int(leftOdom.get()));
            master.set_text(2, 0, values);
            c::task_delay(posDelay);
        }
    }

    void testOdom()
    {
        PIDMove(0, 24, false);
        PIDMove(12, 12, false);        
    }

    void testOdom3(){
        PIDMove(0, 24, true);
        PIDMove(12, 12, true);        
    }

    void testOdom2()
    {
        PIDTurn(90);
        delay(1000);
        PIDTurn(180);
        delay(1000);
        PIDTurn(270);
        delay(1000);
        PIDTurn(720);
    }

    // set current position of bongo
    void setPos(double x, double y)
    {
        X = x;
        Y = y;
    }

    void setRotation(double r){
        //rotation = r;
        Vincent.set_rotation(r);
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
*/
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
            Claw.move_relative(-400, 100);
            Pneumatics.clawRelease();
            //race
            moveForwardTimed(1.4);
            Pneumatics.clawGrab();
            //drop turn lift manuver
            totalForwardMovement = 0;
            moveBackwardTimed(1.60);
            //check for tugging
            while(totalForwardMovement > -20){
                moveBackwardTimed(.25);
            }
            Pneumatics.clawRelease();
            moveBackwardTimed(.15);
            Lift.liftUp();
            delay(1500);
            Lift.stopAll();
            //sus code
            PIDTurn(rotation - 100);
            //strafe left to wall
            strafeTimed(1.4, false, 50);
            //right
            strafeTimed(.4, true, 50);
            
            //rotate and grab
            Pneumatics.toggletilt();
            
            moveBackwardTimed(1, 30);
            Pneumatics.toggleRingles();
            Pneumatics.toggleBack();
            delay(100);
            Pneumatics.toggletilt();
            moveForwardTimed(2, 30);
            moveBackwardTimed(2, 30);
            moveForwardTimed(2, 30);
            moveBackwardTimed(2, 30);
        } else { //right small
            Claw.move_relative(-400, 100);
            Pneumatics.clawRelease();
            //forward
            moveForwardTimed(1.3);
            Pneumatics.clawGrab();

            totalForwardMovement = 0;
            moveBackwardTimed(.3);

            //check for tugging
            while(totalForwardMovement > -4){
                moveBackwardTimed(.25);
            }
            
            //delay(1000000); //check rotation

            //turn and drop
            //PIDTurn(-90);

            //double turnTime = .6;
            setRotation(0);
            PIDTurn(-100);
            //turnTimed(turnTime,  false, 50); //turn right
            Pneumatics.clawRelease();
            Lift.liftUp();

            //rotate and grab
            Pneumatics.toggletilt();
            Pneumatics.toggleRingles();
            moveBackwardTimed(1.5, 50);
            Pneumatics.toggleBack();
            Pneumatics.toggletilt();
            
            //move to line up 
            setRotation(-90);
            PIDTurn(0);
            Lift.stopAll();
            //turnTimed(turnTime,  true, 50); //turn left
            strafeTimed(1.4, true, 100); //right
            strafeTimed(1, false, 50); //left

            //lined up with the ringles
            Pneumatics.setRingles(true);
            moveForwardTimed(2, 30);

            //point turn to get to home zone and slow for more rings
            setRotation(0);
            PIDTurn(180);
            //turnTimed(2*turnTime,  false, 50); //turn right

            moveForwardTimed(.5, 100);
            Pneumatics.setRingles(true);
            moveForwardTimed(3, 30);
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
        Claw.move_relative(-400, 100);
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
        printf("Starting skills\n");
        //This is skills!!!
        executeData(skills_Data, skills_dataLength, skillsDataTime);
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