#include "task1/R2D2.h"
#include "task1/C3PO.h"


int main(void) {
    r2d2::Robot r2("r2d2", 3.3, 10, 1.2, 4);
    c3po::Robot c3("c3po", 5, 30, 5, 10);

    r2.moveFoward();
    r2.moveBackward();
    r2.stopping();

    c3.moveFoward();
    c3.moveBackward();
    c3.stopping();

    return 0;
}