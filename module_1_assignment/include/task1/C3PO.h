#ifndef C3PO_H
#define C3PO_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;

namespace c3po {
class Robot {
    public:
        Robot(string name, float speed, float weight, float size, int nSensors);
        ~Robot();
        void moveFoward();
        void moveBackward();
        void stopping();

    private:
        string _name;
        float _speed;
        float _weight;
        float _size;
        int _nSensors;
};
}

#endif // C3PO_H