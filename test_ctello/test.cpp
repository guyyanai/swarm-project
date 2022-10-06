#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <ctello.h>

std::shared_ptr<cv::VideoCapture> capture;
std::shared_ptr<cv::Mat> frame;
std::shared_ptr<bool> stop;

void videoDroneThread(std::string &videoPath) {
    // Take frames until stop flag turns into true
    capture = std::make_shared<cv::VideoCapture>(videoPath);
    frame = std::make_shared<cv::Mat>();
    while (!*stop)
        capture->read(*frame);
    capture->release();
}

void keepAlive(ctello::Tello &tello){
   	 while(!*stop){
		tello.SendCommandWithResponse("battery?");
        sleep(10);
	}
}
int main() {
    // Read settings
    std::string settingPath = "../config.json";
    std::ifstream programData(settingPath);
    nlohmann::json data;
    programData >> data;
    programData.close();

    // Connect to drone Wifi
    std::string droneName = data["droneName"];
    std::string commandString = "nmcli c up " + droneName;
    const char *command = commandString.c_str();
    system(command);
    sleep(10); // Wait until Wifi connects
    

    // Connect to video
    ctello::Tello tello; // Create a Tello object
    tello.SendCommandWithResponse("streamon"); // Turn on video stream
    std::string videoPath = data["onlineVideoPath"]; // Get video URL from config
    
    // Tell drone to take off
    tello.SendCommandWithResponse("takeoff");
    sleep(5); // Wait until drone takes off

    // When stop turns into true mission finished
    stop = std::make_shared<bool>(false);

    // Create additional threads
    std::thread t_vid = std::thread(videoDroneThread, std::ref(videoPath));
    std::thread t_heartbeat = std::thread(keepAlive, std::ref(tello));

    // WHY SLEEP?
    sleep(5);

    int waitKey = 0;
    std::cout << "Frame size is: " << *frame->size << std::endl;

    // Move command
    // tello.SendCommand("rc dx dz dy dtheta");
    
    for (int i{0}; i < 100; i++) {
	    cv::Mat frameCopy = *frame;
        cv::imshow("Image", frameCopy);
        cv::waitKey(1);
    }
    *stop = true;
    t_vid.join();
    t_heartbeat.join();
    tello.SendCommandWithResponse("land");
    cv::destroyAllWindows();
    return 0;
}
