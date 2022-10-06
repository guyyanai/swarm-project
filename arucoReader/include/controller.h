//
// Created by Guy Yanai & Tomer Gan on 14/08/22.
//

#ifndef INC_2022SUMMERCOURSE_CONTROLLER_H
#define INC_2022SUMMERCOURSE_CONTROLLER_H

#define LOST_THRESHOLD 100000 // Number of updates until losing parent
#define STAY_IN_PLACE_THRESHOLD 30 // Number of updates until staying in place

#define LATEST_STEPS_SIZE 5

// Maximum distance considered to be 'real' and not an error (abs value)
#define MAX_X 5
#define MAX_Y 5
#define MAX_Z 5

// 'Box' around ideal position considered to be good
#define IDEAL_X_PADDING 0.08
#define IDEAL_Y_PADDING 0.08
#define IDEAL_Z_PADDING 0.08

#define EPSILON 0.0000001

#define PI 3.14159265

#include <iostream>
#include <list>
#include <math.h>
#include <unistd.h>
#include <tellocom.h>
#include <nlohmann/json.hpp>
#include "steps.h"

using std::list;

class controller {
public:
    controller(nlohmann::json& controllerConstants, tellocom& com);
    void processDetectorUpdate(double upDown, double rightLeft, double forward, std::pair<int, bool>& leftOverAngle, bool arucoDetected);

    list<steps> latestSteps;
    int noDetectionCounter = 0; // Amount of consecutive updates no marker was detected

private:
    double idealX;
    double idealY;
    double idealZ;

    int xStepFactor;
    int yStepFactor;
    int zStepFactor;
    int tStepFactor;

    tellocom& com;

    bool isRightFollower = false;
    bool didLand = false; // Did drone finish landing operation
    bool isFirstUpdate = true; // Becomes false after first aruco marker detection

    std::pair<int, bool> lastLeftOverAngle{0, false}; // Value of left-over-angle in last update

    void updateVelocities(double myX, double myY, double myZ, std::pair<int, bool>& leftOverAngle);
    void handleLanding();
};

#endif //INC_2022SUMMERCOURSE_CONTROLLER_H
