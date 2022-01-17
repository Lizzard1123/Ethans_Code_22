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

extern void fillEmpty();

extern void setData(int num, double val);

extern void finalizeData();

extern void printData();

extern void runSegment(int line);

extern void executeSkillsData();
#endif