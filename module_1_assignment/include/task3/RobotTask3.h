#ifndef ROBOTTASK3_H
#define ROBOTTASK3_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;


class RobotTask3 {
    public:
        RobotTask3();
        ~RobotTask3();
        void calibrateSensors();
        void readTemperature();
        void readDistance();
        void sendMyName(vector<string> name);        

    private:
        vector<int> _temperature;
        vector<float> _distance;

        template <typename T>
        void printSensorData(string sensorName, string sensorUnits, vector<T> data);
};

#endif // ROBOTTASK3_H