#ifndef ROBOTTASK2_H
#define ROBOTTASK2_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;


class RobotTask2 {
    public:
        RobotTask2();
        ~RobotTask2();
        void calibrateSensors();
        void readSensorData();

    private:
        vector<int> _temperature;
        vector<float> _distance;
        vector<float> _humidity;
        template <typename T>
        void printSensorData(string sensorName, string sensorUnits, vector<T> data);
};

#endif // ROBOTTASK2_H