#ifndef INC_2022SUMMERCOURSE_TELLOCOM_H
#define INC_2022SUMMERCOURSE_TELLOCOM_H

#include <ctello.h>
#include "steps.h"

using ctello::Tello;

class tellocom {
public:
    tellocom(Tello& drone, bool dummy, bool fly);
    void move(int dx, int dy, int dz, int dtheta);
    void move(steps& s);
    void stop();
    void takeoff();
    void land();
    void streamon();
    void checkbattery();
    void printheight();
private:
    Tello& drone;
    bool dummy;
    bool fly;
};

#endif // INC_2022SUMMERCOURSE_TELLOCOM_H
