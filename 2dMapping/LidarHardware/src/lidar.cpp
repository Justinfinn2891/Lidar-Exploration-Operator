#include <iostream>
#include <string>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <vector>
#include <fstream>
#include <cmath>
#include "../include/motor.h"
#include "../include/coordinates.h"
using namespace sl;

/* GOALS:
1.) We need to put together our lidar camera to try to handle 3d computing
2.) We need to start implementing the coordinates to send to the csv file
3.) We need to get the iterative closest point algorithm working*/



void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester);
void SaveToRawFile(std::vector<Coordinates::raw_data> data);


int main() {

    Coordinates::cartesian coordinate;
    Coordinates coords;
    Coordinates::raw_data dataForFile;



    /////////////////////////////////////
    //Basic Initizlization do not touch//
    /////////////////////////////////////
    std::cout << "Starting LIDAR test..." << std::endl;
    std::string serial_port;
    sl_u32 baudrate = 115200;
    serial_port = "/dev/ttyUSB0";

    std::cout << "Using port: " << serial_port << " at baud " << baudrate << std::endl;

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create LIDAR driver." << std::endl;
        return -1;
    }

    IChannel* channel = (*createSerialPortChannel(serial_port.c_str(), baudrate));
    if (!channel) {
        std::cerr << "Failed to create serial channel." << std::endl;
        return -1;
    }

    if (SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "Error: cannot connect to LIDAR on " << serial_port << std::endl;
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    if (SL_IS_FAIL(drv->getDeviceInfo(devinfo))) {
        std::cerr << "Failed to get device info." << std::endl;
        return -1;
    }

    std::cout << "LIDAR connected. Firmware: "
              << (devinfo.firmware_version >> 8) << "."
              << (devinfo.firmware_version & 0xFF)
              << "  Hardware: " << (int)devinfo.hardware_version
              << std::endl;


    if (SL_IS_FAIL(drv->startScan(0, 1))) {
        std::cerr << "Failed to start scan." << std::endl;
        return -1;
    }

    /////////////////////////////////////
    //Basic Initizlization do not touch//
    /////////////////////////////////////


    // ---begin interactive loop--- //

    char command;
    bool first_write = true;
    
    const float min_distance = 0.0f;
    const float max_distance = 250.0f;
    const float min_angle = 0.0f;
    const float max_angle = 2500.0f;

    float testHorizontalAngle = 0;

    do{
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        std::vector<Coordinates::cartesian> finished_points;
        std::vector<Coordinates::raw_data> finished_data;
        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);

            for (size_t i = 0; i < count; ++i) {
                
                float verticalAngle = (nodes[i].angle_z_q14 * 90.f) / 16384.f; // azimuth from lidar
                verticalAngle *= M_PI / 180.0f;
           
                float horizRad = testHorizontalAngle * M_PI / 180.0f;  

                float dist  = nodes[i].dist_mm_q2 / 4.0f;

                dataForFile.angleV = testHorizontalAngle;
                dataForFile.angleH = nodes[i].angle_z_q14;
                dataForFile.distance = nodes[i].dist_mm_q2;


                coordinate.x_coordinate = coords.findX(verticalAngle, dist);
                coordinate.y_coordinate = coords.findY(verticalAngle,horizRad, dist);
                coordinate.z_coordinate = coords.findZ(verticalAngle,horizRad, dist);

                        
                if(coordinate.x_coordinate == 0 && coordinate.y_coordinate == 0 && coordinate.z_coordinate == 0) continue;
                if(coordinate.x_coordinate == NULL && coordinate.y_coordinate == NULL && coordinate.z_coordinate == NULL) continue;
                
                finished_data.push_back(dataForFile);
                finished_points.push_back(coordinate);
            }
        }
        else {
        std::cerr << "Failed to grab scan data." << std::endl;
    }

    SaveToRawFile(finished_data);
    saveToFile(finished_points,first_write);
    first_write = false;
    testHorizontalAngle += 1;   // just to increment each scan for simple testing

    std::cout << "Scan saved. Type 'c' to continue, 'q' to quit: ";
    std::cin >> command;
    } while(command == 'c');
    

    drv->stop();
    drv->setMotorSpeed(0);

    if (drv) {
        delete drv;
    }

    return 0;
}


//Attemps to create or open a csv file for storing the refind points
void saveToFile(std::vector<Coordinates::cartesian> points, bool write_tester){
    std::string file_name = "sorted_xyz.csv";
    std::ofstream file(file_name, std::ios::app);

    if(!file.is_open()){
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
    }

    if(write_tester){
        file << "x,y,z\n";
    }

    for(const auto& p: points){
        file << p.x_coordinate << "," << p.y_coordinate << "," << p.z_coordinate << std::endl; 
    }
}


void SaveToRawFile(std::vector<Coordinates::raw_data> data){
    std::string file_name = "raw_lidar.csv";
    std::ofstream file(file_name, std::ios::app);

    if(!file.is_open()){
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
    }

    for(const auto& p: data){
        file << p.angleV << "," << p.angleH << "," << p.distance << std::endl; 
    }
}
