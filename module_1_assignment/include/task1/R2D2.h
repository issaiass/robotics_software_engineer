#ifndef R2D2_H
#define R2D2_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;

namespace r2d2 {
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

#endif // R2D2_H