#include "task2/RobotTask2.h"


int main(void) {
    RobotTask2 robot;

    robot.calibrateSensors();
    robot.readSensorData();

    return 0;
}