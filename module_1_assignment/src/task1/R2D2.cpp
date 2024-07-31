
#include "task1/R2D2.h"

namespace r2d2 {

Robot::Robot(string name, float speed, float weight, float size, int nSensors) : _name(name), _speed(speed), _weight(weight), _size(size), _nSensors(nSensors) {
    cout << "Robot Initialized as " << name << endl;
}

void Robot::moveFoward() {
    cout << "Moving Forwarda at " << _speed << " m/s" << endl;
}

void Robot::moveBackward() {
    cout << "Moving Backwards at " << -_speed << " m/s" << endl;
}

void Robot::stopping() {
    cout << "Robot stops " <<  endl;
}

Robot::~Robot() {
    cout << "Cleaned robot" << endl;
}

}