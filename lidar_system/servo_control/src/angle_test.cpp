#include <cmath>
#include <algorithm>
#include <iostream>
constexpr float PI = 3.14159265358979323846f;
void angle_transform(float* pan_tilt_angles, float* drone_position, float* object_position)
{
    float xd_ = object_position[0] - drone_position[0]; // diff in x
    float yd_ = object_position[1] - drone_position[1]; // diff in y
    float zd_ = object_position[2] - drone_position[2]; // diff in z

    float horizontal_angle = atan2(yd_, xd_) * (180 / PI); //degrees

    std::cout << "THETA: " << horizontal_angle << std::endl;
    
    float magnitude = std::sqrt(xd_ * xd_ + yd_ * yd_ + zd_ * zd_);

    float vertical_angle = std::clamp(asin(zd_ / magnitude) * (180 / PI), -45.0f, 135.0f);

    std::cout << "PHI: " << vertical_angle << std::endl;
}

int main(){

    float drone_position[] = {0.0f, 0.0f, 0.0f}; //in meters
    float object_position[] = {1.0f, 1.0f, 1.0f}; //in meters
    angle_transform(nullptr, drone_position, object_position);

    return 0;
}    
