#include "tellocom.h"
#include <string>

using std::to_string;

tellocom::tellocom(Tello& drone, bool dummy, bool fly) : drone(drone), dummy(dummy), fly(fly) {}

void tellocom::move(int dx, int dy, int dz, int dtheta) {
	if (dummy || !fly) return;
	drone.SendCommand("rc " + to_string(dx) + " " + to_string(dz) + " " + to_string(dy) + " " + to_string(dtheta));
    usleep(100000); // 0.1 seconds
}

void tellocom::move(steps& s) { move(s.stepX, s.stepY, s.stepZ, s.stepT); }

void tellocom::takeoff() {
	if (dummy || !fly) return;
	drone.SendCommandWithResponse("takeoff");
    sleep(5);
}

void tellocom::land() {
	if (dummy || !fly) return;
	drone.SendCommandWithResponse("land");
    sleep(1);
}

void tellocom::streamon() {
	if (dummy) return;
	drone.SendCommandWithResponse("streamon");
    sleep(5);
}

void tellocom::checkbattery() {
	if (dummy) return;
	drone.SendCommandWithResponse("battery?");
}

void tellocom::stop() {
	if (dummy || !fly) return;
	move(0, 0, 0, 0);
}

void tellocom::printheight() {
	if (dummy || !fly) return;
	drone.SendCommandWithResponse("height?");
}
