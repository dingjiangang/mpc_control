#ifndef SRC_VEHICLE_H
#define SRC_VEHICLE_H
#include <Eigen/Dense>

class Vehicle {
private:
    double dT = 0.01;
    double x;
    double y;
    double speed;
    double acc;
    double steer;
    Eigen::Matrix<double, 4,1> xState;
private:
    double Cr = 90000;
    double Cf = 150000;
    double m  = 2500;
    double Lr = 1.5;
    double Lf = 0.8;
    double Iz = 3000;
public:
    Vehicle();
    double getX() const;
    double getY() const;
    double getHeading() const;
    double getHeadingRate() const;
    double getSpeed() const;
    double getAcc() const;
    void setAcc(double acc);
    void setSteer(double steer);
    void setInitPose(double x_, double y_, double heading_);
    void update();
    void setSimDt(double dt);
    void setInitSpeed(double speed_);
};


#endif //SRC_VEHICLE_H
