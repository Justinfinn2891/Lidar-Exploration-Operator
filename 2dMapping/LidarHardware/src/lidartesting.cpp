#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <sstream>
#include <cstdlib>  // for system()
// Keeping std for testing purposes
using namespace std;

/* Purpose of this script is to help fix our cartesian math issue. 
We will grab data from previous scans to see if we can accurately plot a 3d plane */
const int MAX = 10000;

struct cartesian{
    float x_coordinate, y_coordinate, z_coordinate;
};

float findX(const float &horizontalAngle, const float &verticleAngle, const float &distance);
float findY(const float &horizontalAngle, const float &verticleAngle, const float &distance);
float findZ(const float &verticleAngle, const float &distance);
void saveToFile(std::vector<cartesian> points);

int main(){

std::ifstream file("../tests/raw_lidar.csv");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file." << std::endl;
        return 1;
    }

    int i = 0;
    float angle[MAX]; 
    float distance[MAX];
    float angleV[MAX];

    std::string line;
    std::cout << "Loading the file, extracting data now..." << std::endl;

    while (std::getline(file, line) && i < MAX) {
        std::stringstream ss(line);
        std::string angleStr, distStr, anglVStr;
        // read 3 columns
        if (std::getline(ss, angleStr, ',') && 
            std::getline(ss, distStr, ',') && 
            std::getline(ss, anglVStr, ',')) {
            
            angle[i]   = std::stof(angleStr);
            distance[i] = std::stof(distStr);
            angleV[i]  = std::stof(anglVStr);
            i++;
        }
    }
    

    cout << "data extraction: COMPLETE" << endl; 
    cout << "I IS: " << i << endl;
    vector <cartesian> finished_points;
    for(int j = 0; j < i; j++) {
        float horizRad = (angle[j] * 90.f) / 16384.f; 
        horizRad *= M_PI / 180.0f;

        float verticalAngle = angleV[j] * M_PI / 180.0f;  

        float dist = distance[j] / 4.0f;

        cartesian coordinate;d
        coordinate.x_coordinate = findX(horizRad, verticalAngle, dist);
        coordinate.y_coordinate = findY(horizRad, verticalAngle, dist);
        coordinate.z_coordinate = findZ(verticalAngle, ist);


        cout << "This is the value: " << coordinate.x_coordinate << endl; 
       // if(coordinate.x_coordinate == 0 && coordinate.y_coordinate == 0 && coordinate.z_coordinate == 0) continue;
       // if(coordinate.x_coordinate == NULL && coordinate.y_coordinate == NULL && coordinate.z_coordinate == NULL) continue;
        
        finished_points.push_back(coordinate);
    
    }
    saveToFile(finished_points);
    file.close();
    return 0;
}


/* All of this stuff remains the same from the other .cpp file */
//Find the x based on two angles and a distance
float findX(const float &horizontalAngle, const float &verticleAngle, const float &distance){
    return (distance * cos(verticleAngle) * cos(horizontalAngle));
}

//Find the y based on two angles and a distance
float findY(const float &horizontalAngle, const float &verticleAngle, const float &distance){
    return (distance * cos(verticleAngle) * sin(horizontalAngle)); 
}

//Find the z based on one angle and a distance
float findZ(const float &verticleAngle, const float &distance){
    return (distance * sin(verticleAngle));
}



//Attemps to create or open a csv file for storing the refind points
void saveToFile(std::vector<cartesian> points){
    cout << "DAWD" << endl; 
    std::string file_name = "test.csv";
    std::ofstream file(file_name);
    cout << "IT SAVED";
    if(!file.is_open()){
        std::cerr << "The file has failed to open; possibly failed" << std::endl;
        std::cout << "Filename tried: " << file_name << std::endl;
    }


    for(const auto& p: points){
        file << p.x_coordinate << "," << p.y_coordinate << "," << p.z_coordinate << std::endl; 
    }

    file.close();
}