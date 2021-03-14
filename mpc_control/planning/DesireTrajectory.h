#ifndef MPC_CONTROL_DESIRETRAJECTORY_H
#define MPC_CONTROL_DESIRETRAJECTORY_H
#include <vector>
struct TrajectoryPoint
{
    double x;
    double y;
    double phi;
    double Curvature;
    double v;
    double t;
};

class DesireTrajectory {
public:
    std::vector<TrajectoryPoint> path;
public:
    DesireTrajectory(){};
    void SetDemoTrajData();
    TrajectoryPoint QueryPathPointAtTime(double time);
};


#endif //MPC_CONTROL_DESIRETRAJECTORY_H
