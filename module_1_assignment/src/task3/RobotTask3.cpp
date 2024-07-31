#include "task3/RobotTask3.h"

RobotTask3::RobotTask3() {
    cout << "Robot Initialized" << endl;
}

void RobotTask3::calibrateSensors() {
    _temperature = {29, 26, 25, 23};
    _distance = {3.2, 3.0, 2.9, 1.4};
    cout << "Sensor Data Calibrated" << endl;
}

void RobotTask3::readTemperature() {
    printSensorData("TEMPERATURE", "°C", _temperature);     
}

void RobotTask3::readDistance() {
    printSensorData("DISTANCE", "mm", _distance);     
}

void RobotTask3::sendMyName(vector<string> name) {
    printSensorData("Hello user", "biri biri", name);     
}

template <typename T>
void RobotTask3::printSensorData(string sensorName, string sensorUnits, vector<T> data) {
    cout << "[" << sensorName << " DATA]" << endl;
    for (T d : data) {
        cout << d << " " << sensorUnits <<  ", ";
    }
    cout << endl << endl;
}

RobotTask3::~RobotTask3() {
    cout << "Cleaned robot" << endl;
}