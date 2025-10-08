#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <thread>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "../include/motor.h"
#include "../include/coordinates.h"

using namespace sl;

void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester);
void SaveToRawFile(std::vector<Coordinates::raw_data> data);

int main() {
    Coordinates coords;
    Coordinates::cartesian coordinate;
    Coordinates::raw_data dataForFile;

    /////////////////////////////////////
    // Basic Initialization
    /////////////////////////////////////
    std::cout << "Starting LIDAR test..." << std::endl;
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
    // Motor + Scan Loop
    /////////////////////////////////////

    bool first_write = true;
    char command;
    Motor motor("gpiochip0", 20, 21);  // dirPin=20, stepPin=21

    // ===== Stepper Motor Configuration =====
    const float stepPerRev = 200.0f;    // 1.8° stepper motor
    const float microstep = 16.0f;      // Microstepping setting
    const float gearRatio = 1.0f;       // If no gearing, keep 1.0
    const float degreesPerStep = 360.0f / (stepPerRev * microstep * gearRatio);

    const int stepsPerMove = 3000;      // Your chosen move in microsteps
    const int delay_us = 1000;
    const float moveDegrees = stepsPerMove * degreesPerStep; // Actual degrees per move

    float currentHorizontalAngle = 0.0f;

    std::vector<Coordinates::cartesian> finished_points;
    std::vector<Coordinates::raw_data> finished_data;

    do {
        // Rotate motor by defined step amount
        motor.setDirection(true);
        motor.rotateDegrees(stepsPerMove, delay_us);

        currentHorizontalAngle += moveDegrees;
        if (currentHorizontalAngle >= 360.0f)
            currentHorizontalAngle -= 360.0f;

        // Let system settle after rotation before scanning
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);

            for (size_t i = 0; i < count; ++i) {
                float verticalAngle = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
                verticalAngle *= M_PI / 180.0f;

                float horizRad = currentHorizontalAngle * M_PI / 180.0f;
                float dist = nodes[i].dist_mm_q2 / 4.0f;

                dataForFile.angleV = verticalAngle;
                dataForFile.angleH = currentHorizontalAngle;
                dataForFile.distance = dist;

                coordinate.x_coordinate = coords.findX(verticalAngle, dist);
                coordinate.y_coordinate = coords.findY(verticalAngle, horizRad, dist);
                coordinate.z_coordinate = coords.findZ(verticalAngle, horizRad, dist);

                // Skip invalid points
                if (coordinate.x_coordinate == 0 && coordinate.y_coordinate == 0 && coordinate.z_coordinate == 0)
                    continue;

                finished_data.push_back(dataForFile);
                finished_points.push_back(coordinate);
            }

            std::cout << "Captured " << count << " points at angle: "
                      << currentHorizontalAngle << "°" << std::endl;

        } else {
            std::cerr << "Failed to grab scan data." << std::endl;
        }

        std::cout << "Press 'c' to stop scan or any other key to continue: ";
        std::cin >> command;

    } while (command != 'c' && command != 'C');

    SaveToRawFile(finished_data);
    saveToFile(finished_points, first_write);

    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;

    std::cout << "Scan complete. Data saved." << std::endl;
    return 0;
}


// Attempts to create or open a csv file for storing the refined points
void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester) {
    std::string file_name = "sorted_xyz.csv";
    std::ofstream file(file_name, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
        return;
    }

    if (write_tester) {
        file << "x,y,z\n";
    }

    for (const auto& p : points) {
        file << p.x_coordinate << "," << p.y_coordinate << "," << p.z_coordinate << std::endl;
    }
}


void SaveToRawFile(std::vector<Coordinates::raw_data> data) {
    std::string file_name = "raw_lidar.csv";
    std::ofstream file(file_name, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
        return;
    }

    for (const auto& p : data) {
        file << p.angleV << "," << p.angleH << "," << p.distance << std::endl;
    }
}
