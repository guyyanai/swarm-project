#include "../include/controller.h"

using std::cout;
using std::endl;
using std::pair;
using std::sqrt;

controller::controller(nlohmann::json& controllerConstants, tellocom& com) : idealX(controllerConstants["idealX"]), idealY(controllerConstants["idealY"]),
                                                                             idealZ(-double(controllerConstants["idealZ"])), xStepFactor(controllerConstants["xStepFactor"]),
                                                                             yStepFactor(controllerConstants["yStepFactor"]), zStepFactor(controllerConstants["zStepFactor"]),
                                                                             tStepFactor(controllerConstants["tStepFactor"]), com(com) {}

double absqrt(double x) { return x > 0 ? sqrt(x) : -(sqrt(-x)); }

void controller::processDetectorUpdate(double upDown, double rightLeft, double forward, pair<int, bool>& LOA, bool arucoDetected) {
	// Calculate my position vector in relation to the parent drone as in (0, 0, 0)
	double myX = rightLeft;
	double myY = upDown;
	double myZ = -forward;

	// Sometimes there is a misleading (myX, myY, myZ) = (0, 0, 0), so ignore this case
	if (abs(myX + myY + myZ) < EPSILON)
		arucoDetected = false;

	// Sometimes the aruco marker is "detected" extremely far away, so ignore this case
	if (abs(myX) > MAX_X || abs(myX) > MAX_Y || abs(myZ) > MAX_Z)
		arucoDetected = false;
	
	// Sometimes the aruco marke is "behind" the camera, so ignore this case
	if (myZ > EPSILON)
		arucoDetected = false;

	// If the drone had landed, it doesn't need to move anymore
	if (didLand)
		return;

	// Checking if no markers were detected mid-air
	if (!arucoDetected && !isFirstUpdate) {
		noDetectionCounter++; // Incrementing the no-detection counter

		if (noDetectionCounter > LOST_THRESHOLD)
			handleLanding();
		
		if (noDetectionCounter == STAY_IN_PLACE_THRESHOLD)
			com.stop();

		usleep(10000); // Sleep for 10 milliseconds

		return;
	}
	
	if (!arucoDetected)
		return;

	// Otherwise, marker is detected so we reset no-detection counter
	noDetectionCounter = 0;

	// On first camera update, check if drone is a right or a left follower
	if (isFirstUpdate) {
		isFirstUpdate = false;
		isRightFollower = myX > 0;
		idealX = isRightFollower ? idealX : -idealX;
	}

	// Update velocities on each axis
	updateVelocities(myX, myY, myZ, LOA);
	
	usleep(200000); // Sleep for 200 milliseconds
}

void controller::updateVelocities(double myX, double myY, double myZ, pair<int, bool>& LOA) {
	// Calculate delta vector with the parent's coordinate system
	double deltaX = idealX - myX;
	double deltaY = idealY - myY;
	double deltaZ = idealZ - myZ;
	double deltaT = LOA.first * (LOA.second ? 1 : -1) * PI / 180;
	
	// Calculate delta vector with the current drone's coordinate system
	double moveX = cos(deltaT) * deltaX + sin(deltaT) * deltaZ;
	double moveY = deltaY;
	double moveZ = -sin(deltaT) * deltaX + cos(deltaT) * deltaZ;
	double moveT = deltaT;

	// Normalize values to balance out Tello drone's anomaly
	double telloNormalizedX = absqrt(moveX);
	double telloNormalizedY = absqrt(moveY);
	double telloNormalizedZ = absqrt(moveZ);
	double telloNormalizedT = moveT;
	
	// Calculate stepping vector according to drone's delta vector
	int stepX = telloNormalizedX * xStepFactor * sqrt(-myZ);
	int stepY = telloNormalizedY * yStepFactor;
	int stepZ = telloNormalizedZ * zStepFactor;
	int stepT = telloNormalizedT * tStepFactor * sqrt(-myZ);

	steps curSteps(stepX, stepY, stepZ, stepT);
	latestSteps.push_front(curSteps);
	if (latestSteps.size() > LATEST_STEPS_SIZE)
		latestSteps.pop_back();

    // Check if the drone is in a "good" position according to ideal (x,y,z) and padding
    bool goodX = (abs(deltaX) < IDEAL_X_PADDING);
    bool goodY = (abs(deltaY) < IDEAL_Y_PADDING);
    bool goodZ = (abs(deltaZ) < IDEAL_Z_PADDING);
	bool good = goodX & goodY & goodZ;

	// Start logging current detection
	cout << endl << "--------------------------------------------------------------" << endl;
	
	// Log results
	cout << "LeftOverAngle: (leftOverAngle, clockwise) = (" << LOA.first << ", " << LOA.second << ")" << endl;
	cout << std::showpos; // Show '+' on positive
    cout << "Ideal position:  (idealX, idealY, idealZ) = (" << idealX << ", " << idealY << ", " << idealZ << ")" << endl;
	cout << "My position:     (myX, myY, myZ)          = (" << myX    << ", " << myY    << ", " << myZ    << ")" << endl << endl;
	cout << "Dist from ideal: (deltaX, deltaY, deltaZ) = (" << deltaX << ", " << deltaY << ", " << deltaZ << ")" << endl;
	cout << "Moving vector:   (moveX, moveY, moveZ)    = (" << moveX  << ", " << moveY  << ", " << moveZ  << ")" << endl << endl;
	cout << "Stepping vector: (stepX, stepY, stepZ)    = (" << stepX  << ", " << stepY  << ", " << stepZ  << ")" << endl << endl;
	cout << "Angle from ideal:(deltaT)                 = (" << deltaT << ")" << endl; 
	cout << std::noshowpos;
	cout << "Is good (x,y,z): (goodX, goodY, goodZ)    = (" << goodX  << ", " << goodY  << ", " << goodZ  << ")" << endl;
	
	// Print movement direction helper
	cout << "RL: ";
	double xUnits = IDEAL_X_PADDING / 2;
	for (int i = 0; i <  moveX / xUnits; i++) cout << ">";
	for (int i = 0; i < -moveX / xUnits; i++) cout << "<";
	cout << endl; cout << "UD: ";
	double yUnits = IDEAL_Y_PADDING / 2;
	for (int i = 0; i <  moveY / yUnits; i++) cout << "^";
	for (int i = 0; i < -moveY / yUnits; i++) cout << "v";
	cout << endl; cout << "FB: ";
	double zUnits = IDEAL_Z_PADDING / 2;
	for (int i = 0; i <  moveZ / zUnits; i++) cout << "x";
	for (int i = 0; i < -moveZ / zUnits; i++) cout << "*";
	cout << endl;

	// Print summary
	cout << (isRightFollower ? "[Right Follower]" : "[Left Follower]") << endl;
	cout << (good ? "[V] Good" : "[X] Bad ") << " position. Distance from Ideal: (x, y, z) = (" << deltaX << ", " << deltaY << ", " << deltaZ << ")" << endl;

	// Finish logging
	cout << "--------------------------------------------------------------" << endl;
}

void controller::handleLanding() {
	// Begin landing operation using Tello API
	cout << "Leader is missing. Starting landing." << endl;
	com.land();
	cout << "Landing complete, mission finished successfully." << endl;
	
	// Update flags
	didLand = true;
}
