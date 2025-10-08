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
    std::cout << "Starting 360° LiDAR sweep..." << std::endl;
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
    Motor motor("gpiochip0", 20, 21);  // dirPin=20, stepPin=21

    // ===== Stepper Motor Configuration =====
    const float stepPerRev = 200.0f;    // 1.8° per full step
    const float microstep = 16.0f;      // Match TB6600 DIP switches
    const float gearRatio = 1.0f;       // 1.0 if no gears
    const float degreesPerStep = 360.0f / (stepPerRev * microstep * gearRatio);

    // Target horizontal resolution
    const float moveDegrees = 1.0f;  // desired degrees per sweep increment
    const int delay_us = 800;        // step delay for motor (smoothness)
    const int stepsPerMove = static_cast<int>(moveDegrees / degreesPerStep);

    float currentHorizontalAngle = 0.0f;
    std::vector<Coordinates::cartesian> finished_points;
    std::vector<Coordinates::raw_data> finished_data;

    std::cout << "Sweeping full 360° in " << moveDegrees << "° increments..." << std::endl;

    while (currentHorizontalAngle < 360.0f) {
        motor.setDirection(true);
        motor.rotateSteps(stepsPerMove, delay_us);

        // Let vibrations settle before capture
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        // Capture multiple frames per step to densify data
        for (int frame = 0; frame < 3; ++frame) {
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

                    if (coordinate.x_coordinate == 0 && coordinate.y_coordinate == 0 && coordinate.z_coordinate == 0)
                        continue;

                    finished_data.push_back(dataForFile);
                    finished_points.push_back(coordinate);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cout << "Captured " << count * 3 << " points at " 
                  << currentHorizontalAngle << "°" << std::endl;

        currentHorizontalAngle += moveDegrees;
    }

    SaveToRawFile(finished_data);
    saveToFile(finished_points, true);

    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;

    std::cout << "Full 360° scan complete. Data saved." << std::endl;
    return 0;
}


// ===== Save XYZ Points =====
void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester) {
    std::string file_name = "sorted_xyz.csv";
    std::ofstream file(file_name, std::ios::out);

    if (!file.is_open()) {
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
        return;
    }

    if (write_tester)
        file << "x,y,z\n";

    for (const auto& p : points)
        file << p.x_coordinate << "," << p.y_coordinate << "," << p.z_coordinate << "\n";
}


// ===== Save Raw Lidar Data =====
void SaveToRawFile(std::vector<Coordinates::raw_data> data) {
    std::string file_name = "raw_lidar.csv";
    std::ofstream file(file_name, std::ios::out);

    if (!file.is_open()) {
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
        return;
    }

    for (const auto& p : data)
        file << p.angleV << "," << p.angleH << "," << p.distance << "\n";
}
