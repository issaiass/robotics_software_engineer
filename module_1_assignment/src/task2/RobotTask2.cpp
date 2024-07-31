#include "task2/RobotTask2.h"

RobotTask2::RobotTask2() {
    cout << "Robot Initialized" << endl;
}

void RobotTask2::calibrateSensors() {
    _temperature = {29, 26, 25, 23};
    _distance = {3.2, 3.0, 2.9, 1.4};
    _humidity = {50, 70, 90, 95, 98, 96};
    cout << "Sensor Data Calibrated" << endl;
}

void RobotTask2::readSensorData() {
    printSensorData("TEMPERATURE", "°C", _temperature);
    printSensorData("DISTANCE", "mm", _distance);
    printSensorData("HUMIDITY", "%HR", _humidity);        
}

template <typename T>
void RobotTask2::printSensorData(string sensorName, string sensorUnits, vector<T> data) {
    cout << "[" << sensorName << " DATA]" << endl;
    for (T d : data) {
        cout << d << " " << sensorUnits <<  ", ";
    }
    cout << endl << endl;
}

RobotTask2::~RobotTask2() {
    cout << "Cleaned robot" << endl;
}