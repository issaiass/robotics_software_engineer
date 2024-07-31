#include "task3/RobotTask3.h"


int main(void) {
    RobotTask3 robot;

    robot.calibrateSensors();
    robot.readTemperature();
    robot.readDistance();
    vector<string> name = {"Hello", "i am", "a defective", "robot"};
    robot.sendMyName(name);        


    return 0;
}