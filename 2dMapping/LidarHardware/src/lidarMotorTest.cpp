#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <thread>
#include <atomic>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "../include/motor.h"
#include "../include/coordinates.h"

using namespace sl;

void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester);
void SaveToRawFile(std::vector<Coordinates::raw_data> data);

std::atomic<bool> keepRunning(true);

// ==============================
// Continuous Rotation Thread
// ==============================
void continuousRotate(Motor* motor, int delay_us, std::atomic<float>& horizontalAngle,
                      float degreesPerStep) {
    while (keepRunning) {
        motor->step(delay_us);
        horizontalAngle += degreesPerStep;
        if (horizontalAngle >= 360.0f) horizontalAngle -= 360.0f;
    }
}

int main() {
    Coordinates coords;
    Coordinates::cartesian coordinate;
    Coordinates::raw_data dataForFile;

    /////////////////////////////////////
    // Basic Initialization
    /////////////////////////////////////
    std::cout << "Starting continuous LiDAR scan..." << std::endl;
    std::string serial_port = "/dev/ttyUSB0";
    sl_u32 baudrate = 115200;

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create LIDAR driver." << std::endl;
        return -1;
    }

    IChannel* channel = (*createSerialPortChannel(serial_port.c_str(), baudrate));
    if (!channel || SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "Error: cannot connect to LIDAR on " << serial_port << std::endl;
        delete drv;
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    if (SL_IS_FAIL(drv->getDeviceInfo(devinfo))) {
        std::cerr << "Failed to get device info." << std::endl;
        delete drv;
        return -1;
    }

    std::cout << "LIDAR connected. Firmware: "
              << (devinfo.firmware_version >> 8) << "."
              << (devinfo.firmware_version & 0xFF)
              << "  Hardware: " << (int)devinfo.hardware_version
              << std::endl;

    if (SL_IS_FAIL(drv->startScan(0, 1))) {
        std::cerr << "Failed to start scan." << std::endl;
        delete drv;
        return -1;
    }

    /////////////////////////////////////
    // Motor Configuration
    /////////////////////////////////////
    Motor motor("gpiochip0", 20, 21); // dirPin=20, stepPin=21
    motor.setDirection(true);

    const float stepPerRev = 200.0f;     // 1.8° stepper
    const float microstep = 16.0f;       // TB6600 DIP: 1/16 step
    const float gearRatio = 1.0f;        // 1:1
    const float degreesPerStep = 360.0f / (stepPerRev * microstep * gearRatio);

    const int delay_us = 700;            // adjust speed here (~rpm)
    std::atomic<float> currentHorizontalAngle(0.0f);

    // Start rotation thread
    std::thread motorThread(continuousRotate, &motor, delay_us,
                            std::ref(currentHorizontalAngle), degreesPerStep);

    /////////////////////////////////////
    // Continuous LiDAR Capture Loop
    /////////////////////////////////////
    std::vector<Coordinates::cartesian> finished_points;
    std::vector<Coordinates::raw_data> finished_data;

    std::cout << "Press 'c' to stop scanning." << std::endl;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    while (true) {
        if (std::cin.peek() == 'c') {
            std::cin.ignore();
            break;
        }

        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);

            float horizAngleSnapshot = currentHorizontalAngle.load();

            for (size_t i = 0; i < count; ++i) {
                float verticalAngle = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
                verticalAngle *= M_PI / 180.0f;

                float horizRad = horizAngleSnapshot * M_PI / 180.0f;
                float dist = nodes[i].dist_mm_q2 / 4.0f;

                dataForFile.angleV = verticalAngle;
                dataForFile.angleH = horizAngleSnapshot;
                dataForFile.distance = dist;

                coordinate.x_coordinate = coords.findX(verticalAngle, dist);
                coordinate.y_coordinate = coords.findY(verticalAngle, horizRad, dist);
                coordinate.z_coordinate = coords.findZ(verticalAngle, horizRad, dist);

                if (coordinate.x_coordinate == 0 && coordinate.y_coordinate == 0 && coordinate.z_coordinate == 0)
                    continue;

                finished_data.push_back(dataForFile);
                finished_points.push_back(coordinate);
            }
        }
    }

    /////////////////////////////////////
    // Cleanup
    /////////////////////////////////////
    keepRunning = false;
    if (motorThread.joinable()) motorThread.join();

    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;

    SaveToRawFile(finished_data);
    saveToFile(finished_points, true);

    std::cout << "Continuous scan complete. Data saved." << std::endl;
    return 0;
}


// =============================================================
// CSV Writers (unchanged)
// =============================================================
void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester) {
    std::string file_name = "sorted_xyz.csv";
    std::ofstream file(file_name, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_name << std::endl;
        return;
    }

    if (write_tester) file << "x,y,z\n";
    for (const auto& p : points)
        file << p.x_coordinate << "," << p.y_coordinate << "," << p.z_coordinate << "\n";
}

void SaveToRawFile(std::vector<Coordinates::raw_data> data) {
    std::string file_name = "raw_lidar.csv";
    std::ofstream file(file_name, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << file_name << std::endl;
        return;
    }

    for (const auto& p : data)
        file << p.angleV << "," << p.angleH << "," << p.distance << "\n";
}
