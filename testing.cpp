#include <iostream>
#include "depthai/depthai.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include "json/json.h"
#include <math.h>
#include <atomic>
// The highgui include is no longer needed as all GUI calls have been removed.
// #include <opencv2/highgui.hpp> 

using namespace std;
using namespace cv;

#define SERIAL_PORT_PATH "/dev/ttyS0"

struct termios g_tty;
int g_fd;

// Precompute constants for conversion
constexpr double PI = 3.141592653589793238463;
constexpr double TO_RAD = PI / 180.0;
constexpr double TO_DEG = 180.0 / PI;

inline double toRadians(double deg) { return deg * TO_RAD; }
inline double toDegrees(double rad) { return rad * TO_DEG; }

// File operation functions
static int file_open_and_get_descriptor(const char *fname) {
    int fd = open(fname, O_RDWR | O_NONBLOCK);
    if(fd < 0) {
        printf("Could not open file %s...%d\r\n", fname, fd);
    }
    return fd;
}

static inline int file_write_data(int fd, const char *buff, int len_buff) {
    return write(fd, buff, len_buff);
}

static inline int file_read_data(int fd, uint8_t *buff, uint32_t len_buff) {
    return read(fd, buff, len_buff);
}

static inline int file_close(int fd) {
    return close(fd);
}

static void open_serial_port() {
    g_fd = file_open_and_get_descriptor(SERIAL_PORT_PATH);
    if(g_fd < 0) {
        printf("Something went wrong while opening the port...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void configure_serial_port() {
    if(tcgetattr(g_fd, &g_tty)) {
        printf("Something went wrong while getting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }

    cfsetispeed(&g_tty, B2000000);
    cfsetospeed(&g_tty, B2000000);
    cfmakeraw(&g_tty);

    if(tcsetattr(g_fd, TCSANOW, &g_tty)) {
        printf("Something went wrong while setting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void sendData(const string& s) {
    file_write_data(g_fd, s.c_str(), s.length());
}

static void close_serial_port() {
    file_close(g_fd);
}

// Optimized Calculate function using references and avoiding unnecessary operations
void Calculate(double &angle, const Mat &hsv_img, int cntSize, const Scalar &lower, const Scalar &upper,
               bool needsDist, int xoffset, int yoffset, double &dist) {
    Mat maskElement;
    inRange(hsv_img, lower, upper, maskElement);

    std::vector<std::vector<Point>> cnts;
    findContours(maskElement, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (cnts.empty()) return;

    // Find the largest contour
    size_t maxIndex = 0;
    double maxArea = contourArea(cnts[0]);

    for (size_t i = 1; i < cnts.size(); i++) {
        double area = contourArea(cnts[i]);
        if (area > maxArea) {
            maxArea = area;
            maxIndex = i;
        }
    }

    const auto& max = cnts[maxIndex];
    if (contourArea(max) > cntSize) {
        Rect boundRect = boundingRect(max);

        int midx = (boundRect.tl().x + boundRect.br().x) / 2 - xoffset;
        int midy = (boundRect.tl().y + boundRect.br().y) / 2 - yoffset;

        angle = atan2(midx, midy) * TO_DEG + 180.0;

        if (needsDist) {
            dist = sqrt(midx * midx + midy * midy) * 1.5;
        }
    }
}

// Structure to hold detection results for thread safety
struct DetectionResult {
    std::atomic<double> angle{-5};
    std::atomic<double> dist{-5};
};

int main() {
    open_serial_port();
    configure_serial_port();

    // File Reading
    ifstream thresholds("/home/apollo/colortracking/colortrackingCPP/src/thresholds.json");
    if (!thresholds.is_open()) {
        cerr << "Failed to read thresholds: thresholds.json" << endl;
        return 1;
    }
    
    Json::Value readThresholds;
    thresholds >> readThresholds;
    thresholds.close();

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    // Configure camera with JSON settings
    camRgb->initialControl.setManualFocus(readThresholds["focus"].asInt());
    camRgb->initialControl.setManualWhiteBalance(readThresholds["wb"].asInt());
    camRgb->initialControl.setManualExposure(readThresholds["exposure"]["time"].asInt(), readThresholds["exposure"]["sens"].asInt());
    camRgb->initialControl.setContrast(readThresholds["contrast"].asInt());
    camRgb->initialControl.setSaturation(readThresholds["saturation"].asInt());
    camRgb->initialControl.setSharpness(readThresholds["sharpness"].asInt());
    
    xoutVideo->setStreamName("video");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P); // Using supported resolution
    camRgb->setVideoSize(readThresholds["crop"]["width"].asInt(), readThresholds["crop"]["height"].asInt());
    camRgb->setFps(60); // Using a supported FPS for the resolution

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER_PLUS);
    auto video = device.getOutputQueue("video", 1, false);

    // JSON parsing - create thresholds once
    const Scalar lower_range(readThresholds["ball"]["lower"]["H"].asInt(), readThresholds["ball"]["lower"]["S"].asInt(), readThresholds["ball"]["lower"]["V"].asInt());
    const Scalar upper_range(readThresholds["ball"]["upper"]["H"].asInt(), readThresholds["ball"]["upper"]["S"].asInt(), readThresholds["ball"]["upper"]["V"].asInt());
    const Scalar lower_range_yellow(readThresholds["yellowGoal"]["lower"]["H"].asInt(), readThresholds["yellowGoal"]["lower"]["S"].asInt(), readThresholds["yellowGoal"]["lower"]["V"].asInt());
    const Scalar upper_range_yellow(readThresholds["yellowGoal"]["upper"]["H"].asInt(), readThresholds["yellowGoal"]["upper"]["S"].asInt(), readThresholds["yellowGoal"]["upper"]["V"].asInt());
    const Scalar lower_range_blue(readThresholds["blueGoal"]["lower"]["H"].asInt(), readThresholds["blueGoal"]["lower"]["S"].asInt(), readThresholds["blueGoal"]["lower"]["V"].asInt());
    const Scalar upper_range_blue(readThresholds["blueGoal"]["upper"]["H"].asInt(), readThresholds["blueGoal"]["upper"]["S"].asInt(), readThresholds["blueGoal"]["upper"]["V"].asInt());

    const int xoffset = readThresholds["offsets"]["x"].asInt();
    const int yoffset = readThresholds["offsets"]["y"].asInt();

    // Create mask once
    const int maskX = readThresholds["mask1"]["x"].asInt();
    const int maskY = readThresholds["mask1"]["y"].asInt();
    const int maskSize1 = readThresholds["mask1"]["size1"].asInt();
    const int maskSize2 = readThresholds["mask1"]["size2"].asInt();
    
    Mat mask = Mat::zeros(Size(655, 600), CV_8UC1);
    ellipse(mask, Point(maskX, maskY), Size(maskSize1, maskSize2), 0, 0, 360, Scalar(255), -1);

    int prevBall = -1;
    int prevDist = -1;
    double dInput;
    
    // Pre-allocate matrices to avoid reallocations
    Mat original, masked, hsv;
    
    // Prepare thread-safe result objects
    DetectionResult ballResult, yellowResult, blueResult;

    while(true) {
        auto begin = chrono::high_resolution_clock::now();
        
        // Reset values
        double balldist = -5;
        ballResult.angle = -5;
        yellowResult.angle = -5;
        blueResult.angle = -5;

        // Get frame and process
        auto videoIn = video->get<dai::ImgFrame>();
        
        if (original.empty()) {
            original = Mat(600, 655, CV_8UC3);
            masked = Mat(600, 655, CV_8UC3);
            hsv = Mat(600, 655, CV_8UC3);
        }
        
        resize(videoIn->getCvFrame(), original, Size(655, 600), cv::INTER_AREA);
        masked.setTo(Scalar(0,0,0));
        original.copyTo(masked, mask);
        cvtColor(masked, hsv, COLOR_BGR2HSV);

        // Run detection threads
        double ballAngleTemp = -5, yellowAngleTemp = -5, blueAngleTemp = -5;
        
        thread BallThread([&]() {
            Calculate(ballAngleTemp, hsv, 5, lower_range, upper_range, true, xoffset, yoffset, balldist);
            ballResult.angle = ballAngleTemp;
            ballResult.dist = balldist;
        });
        
        thread BlueThread([&]() {
            Calculate(blueAngleTemp, hsv, 400, lower_range_blue, upper_range_blue, false, xoffset, yoffset, balldist);
            blueResult.angle = blueAngleTemp;
        });
        
        thread YellowThread([&]() {
            Calculate(yellowAngleTemp, hsv, 400, lower_range_yellow, upper_range_yellow, false, xoffset, yoffset, balldist);
            yellowResult.angle = yellowAngleTemp;
        });

        BallThread.join();
        BlueThread.join();
        YellowThread.join();

        // Process distance calculation
        double ballAngle = ballResult.angle;
        balldist = ballResult.dist;
        
        if (balldist != -5) {
            if (balldist < 185) {
                balldist = -228.02 * exp(-0.00198188 * balldist) + 200.086;
            } else {
                balldist = 7.01168 * exp(0.00594217 * balldist) + 27.48;
            }
        }

        // Calculate derivative
        double newballAngle = ballAngle > 180 ? (360 - ballAngle) : ballAngle;
        dInput = (sin(toRadians(static_cast<int>(newballAngle))) * static_cast<int>(balldist)) - 
                 (sin(toRadians(prevBall)) * prevDist);
        
        double roundedD = -5;
        if (dInput < 0 && (ballAngle < 90 || ballAngle > 270)) {
            roundedD = round(-dInput * 100) / 100;
        }

        // Print detection results to console for logging
        std::cout << "Ball Angle: " << ballAngle << ", Dist: " << balldist
                  << ", Blue Angle: " << blueResult.angle << ", Yellow Angle: " << yellowResult.angle
                  << ", Derivative: " << roundedD << std::endl;

        // Prepare message with string buffer
        char dataBuffer[128];
        snprintf(dataBuffer, sizeof(dataBuffer), "%db%da%dc%dd%df", 
                 static_cast<int>(ballAngle), 
                 static_cast<int>(balldist),
                 static_cast<int>(blueResult.angle), 
                 static_cast<int>(yellowResult.angle), 
                 static_cast<int>(roundedD));
        
        sendData(dataBuffer);
        
        // Update previous values
        prevBall = newballAngle;
        prevDist = balldist;

        auto end = chrono::high_resolution_clock::now();
        double loop_duration_ms = chrono::duration_cast<chrono::milliseconds>(end - begin).count();
        if (loop_duration_ms > 0) {
            std::cout << "Processing FPS = " << 1000.0 / loop_duration_ms << std::endl;
        }
    }
    
    close_serial_port();
    return 0;
}
