#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;

int main(){

    // Convent P to Homogeneous Coordinates
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
    
    // Rotation matrix 
    Eigen::Matrix3f R;
    float theta = acos(-1) / 4;
    R << cos(theta), -sin(theta), 0,
         sin(theta), cos(theta),  0,
         0,          0,           1;

    // Translation matrix
    Eigen::Matrix3f T;
    T << 1, 0, 1,
         0, 1, 2,
         0, 0, 1;
    

    Eigen::Vector3f ans = T * R * p;
    std::cout <<"(" << ans(0) << " , " << ans(1) << ")"<< std::endl;

    return 0;
}