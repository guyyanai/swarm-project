//
// Created by tzuk on 6/6/22.
//

#ifndef INC_2022SUMMERCOURSE_ARUCO_H
#define INC_2022SUMMERCOURSE_ARUCO_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <thread>

class aruco {
public:
    aruco(std::string &yamlCalibrationPath, int cameraPort, float currentMarkerSize);

    ~aruco();

    aruco(std::string &yamlCalibrationPath, std::string &cameraString, float currentMarkerSize);

    void setHoldCamera(bool value) { *holdCamera = value; };

    void setStop(bool value) { stop = value; };

    void trackMarkerThread();

    double upDown = 0.0;
    double forward = 0.0;
    double rightLeft = 0.0;
    std::pair<int, bool> leftOverAngle{0, false};
    long amountOfUSleepForTrackMarker = 5000;

    bool arucoDetected = false;

private:
    bool runCamera;
    bool stop;
    std::string yamlCalibrationPath;
    std::shared_ptr<cv::Mat> frame;
    float currentMarkerSize;
    std::thread cameraThread;
    std::thread arucoThread;
    std::shared_ptr<bool> holdCamera;
    std::shared_ptr<cv::VideoCapture> capture;


    std::vector<cv::Mat> getCameraCalibration(const std::string &path);

    void getEulerAngles(cv::Mat &rotCameraMatrix, cv::Vec3d &eulerAngles);

    std::pair<int, bool> getLeftOverAngleFromRotationVector(const cv::Vec<double, 3> &rvec);

    void getCameraFeed();
};


#endif //INC_2022SUMMERCOURSE_ARUCO_H
