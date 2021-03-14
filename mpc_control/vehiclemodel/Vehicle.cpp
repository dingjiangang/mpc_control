#include "Vehicle.h"

Vehicle::Vehicle()
{
    double y0 = 0;
    double phi0 = 0;
    xState(0) = y0;
    xState(1) = 0;
    xState(2) = phi0;
    xState(3) = 0;
    speed = 0.01;
    acc = 0;
    steer = 0;
}
double Vehicle::getX() const {
    return x;
}

double Vehicle::getY() const {
    return y;
}

double Vehicle::getHeading() const {
    return xState(2);
}
double Vehicle::getHeadingRate() const {
    return xState(3);
}
double Vehicle::getSpeed() const {
    return speed;
}
double Vehicle::getAcc() const {
    return acc;
}
void Vehicle::setAcc(double acc) {
    Vehicle::acc = acc;
}
void Vehicle::setSteer(double steer) {
    Vehicle::steer = steer;
}
void Vehicle::setSimDt(double dt)
{
    dT = dt;
}
void Vehicle::setInitSpeed(double speed_)
{
    speed = speed_;
}
void Vehicle::setInitPose(double x_, double y_, double heading_)
{
    x = x_;
    y = y_;
    xState(2) = heading_;
}
void Vehicle::update() {
    speed += acc*dT;
    if(speed > 30.0)speed = 30.0;
    if(speed < 0.01)speed = 0.01;
    double phi = xState(2);
    double vy  = xState(1);
    x = x + speed*dT*cos(phi) +vy*dT*cos(phi-M_PI/2);
    y = y + speed*dT*sin(phi) +vy*dT*sin(phi-M_PI/2);;

    Eigen::Matrix<double, 4,4> A = Eigen::Matrix<double, 4,4>::Zero(4,4);
    A(0,1) = 1.0;
    A(1,1) = -2.0*(Cf+Cr)/(m*speed);
    A(1,3) = -speed - (2.0*Cf*Lf - 2.0*Cr*Lr)/(m*speed);
    A(2,3) = 1.0;
    A(3,1) = -(2*Cf*Lf - 2*Cr*Lr)/(Iz*speed);
    A(3,3) = -(2*Cf*Lf*Lf + 2*Cr*Lr*Lr)/(Iz*speed);

    Eigen::Matrix<double, 4,1> B = Eigen::Matrix<double, 4,1>::Zero(4,1);
    B(1) = 2*Cf/m;
    B(3) = 2*Lf*Cf/Iz;
    //Eigen::Matrix<double, 4,1> dx = A*xState + B*steer;
    //xState = xState + dx*dT;
    Eigen::Matrix<double, 4,4> I = Eigen::Matrix<double, 4,4>::Identity(4,4);
    Eigen::Matrix<double, 4,4> Ad = A*dT + I;
    Eigen::Matrix<double, 4,1> Bd = B*dT;

    xState = Ad * xState + Bd *steer;
}
