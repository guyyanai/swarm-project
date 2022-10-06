//
// Created by tzuk on 6/6/22. Updated by Tomer Gan & Guy Yanai on 26/8/22.
//

#include <nlohmann/json.hpp>
#include <unistd.h>
#include "aruco.h"
#include "controller.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <tellocom.h>
#include <signal.h>
#include <stdlib.h>

using std::string;
using std::cout;
using std::endl;
using std::ifstream;
using ctello::Tello;

bool stopKeepAlive = false;
bool stopController = false;
bool stopPrediction = false;

Tello* tellop; // Create a Tello object
tellocom* comp; // Create communication object with Tello

void keepAlive(tellocom& com) {
    // Keep communication with drone alive
    while (!stopKeepAlive) {
        com.checkbattery();
        sleep(10);
    }
}

void computeGuess(controller& droneControl, tellocom& com) {
    // Predict parent drone location and move towards it
    while (!stopPrediction) {
        if (droneControl.noDetectionCounter == 0)
            com.move(droneControl.latestSteps.front());
        else {
            steps avgSteps(0, 0, 0, 0);
            for (steps& s : droneControl.latestSteps)
                avgSteps = avgSteps + s;
            avgSteps = avgSteps / droneControl.latestSteps.size();
            com.move(avgSteps);
        }
    }
}

void controlAruco(aruco& detector, controller& droneControl) {
    while (!stopController) {
        droneControl.processDetectorUpdate(detector.upDown, detector.rightLeft, detector.forward, detector.leftOverAngle, detector.arucoDetected);
    }
}

void runArucoTello(aruco& detector, nlohmann::json& controllerConstants, Tello& tello, tellocom& com) {
    // Tell drone to take off
    com.takeoff();

    // Keep alive thread
    std::thread t_heartbeat = std::thread(keepAlive, std::ref(com));

	// Controller thread
    controller droneControl(controllerConstants, com);
    std::thread t_controller = std::thread(controlAruco, std::ref(detector), std::ref(droneControl));

    // Guess computer thread
    std::thread t_guesser = std::thread(computeGuess, std::ref(droneControl), std::ref(com));

	t_heartbeat.join();
	t_controller.join();
    t_guesser.join();
}

// Testing of Aruco marker detection using USB camera
void runArucoCamera(aruco& detector, nlohmann::json& controllerConstants) {
    // Create controller for drone
    Tello dummytello;
    tellocom dummycom(dummytello, true, false);
    controller droneControl(controllerConstants, dummycom);

    // Execute run-loop
    while (true) {
        droneControl.processDetectorUpdate(detector.upDown, detector.rightLeft, detector.forward, detector.leftOverAngle, detector.arucoDetected);
    }
}

void signal_callback_handler(int signum) {
    // Catch Ctrl+C on terminal and land drones before exit
    tellocom& com = *comp;
    com.stop();
    sleep(1);
    com.land();
    // Force crash application:
    // int a = 5;
    // a /= 0;
    exit(signum);
}

int main(int argc, char **argv) {
    // Load and read configuration file
    cout << "Loading configuration file" << endl;
    ifstream programData("../config.json");
    nlohmann::json data;
    programData >> data;
    programData.close();
    string yamlCalibrationPath = data["yamlCalibrationPath"];
    bool isCameraString = data["isCameraString"];
    string cameraString = data["cameraString"];
    int cameraPort = data["cameraPort"];
    float currentMarkerSize = data["currentMarkerSize"];
    string droneName = data["droneName"];
    if (argc > 1) droneName = string(argv[1]);
    cout << "droneName: " << droneName << endl;
    nlohmann::json controllerConstants = data["controllerConstants"];
    cout << "Configuration file read successfully" << endl;
    
    // Configurate standard out stream
    cout << std::fixed;
    cout << std::showpoint;
    cout << std::setprecision(3);

    signal(SIGINT, &signal_callback_handler);

    // Create aruco detector according to config and start program
    if (isCameraString) {
        // Connect to drone Wifi
        cout << "Connecting to Wifi..." << endl;
        string command = "nmcli c up " + droneName;
        system(command.c_str());
        sleep(10); // Wait until Wifi connects
        cout << "Finished trying to connect to Wifi" << endl;
    
        // Connect to video
        tellop = new Tello();
        Tello& tello = *tellop;
        comp = new tellocom(tello, false, true);
        tellocom& com = *comp;
        cout << "Turning on video stream" << endl;
        com.streamon(); // Turn on stream and wait
        cout << "Finished waiting for video stream to turn on" << endl;

        aruco detector(yamlCalibrationPath, cameraString, currentMarkerSize);
        cout << "Starting runAruco() using camera string:" << endl << cameraString << endl;
        runArucoTello(detector, controllerConstants, tello, com);
    } else {
        aruco detector(yamlCalibrationPath, cameraPort, currentMarkerSize);
        cout << "Starting runAruco() using camera port: " << cameraPort << endl;
        runArucoCamera(detector, controllerConstants);
    }

    delete tellop;
    delete comp;
    return 0;
}
