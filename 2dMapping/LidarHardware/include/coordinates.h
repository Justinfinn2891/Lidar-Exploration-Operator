#ifndef COORDINATES_H
#define COORDINATES_H

class Coordinates{
 
    public:

    float findZ(const float &verticleAngle, const float &horizontalAngle, const float &distance);
    float findY(const float &verticleAngle, const float &horizontalAngle, const float &distance);
    float findX(const float &verticleAngle, const float &distance);

    struct cartesian{
    float x_coordinate, y_coordinate, z_coordinate;
    };

    struct raw_data{
    float distance, angleH, angleV, quality; 
    };

};
#endif //COORDINATES_H