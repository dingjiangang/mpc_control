#include "DesireTrajectory.h"
#include <cmath>
#include "polynomial/QuarticPolynomial.h"
#include "polynomial/QuinticPolynomial.h"
double norm(double x, double y)
{
    return sqrt(x*x+y*y);
}
double getCurvature(double ax,double ay, double bx, double by,double cx, double cy)
{
    double abx = bx - ax;
    double bcx = cx - bx;
    double acx = cx - ax;
    double aby = by - ay;
    double bcy = cy - by;
    double acy = cy - ay;
    double sinC = (acy*bcx - acx*bcy)/norm(acx,acy)/norm(bcx,bcy);
    return 2*sinC/norm(abx,aby);
}
void DesireTrajectory::SetDemoTrajData()
{
    path.clear();
    QuarticPolynomial lon(0, 10.0, 0.0, 10.0, 0.0, 6.0);
    QuinticPolynomial leftlat(0, 0, 0, 4, 0, 0, 6.0);
    QuinticPolynomial rightlat(0, 0, 0, -4, 0, 0, 6.0);
    double t = 0.0;
    for(int i = 0; i < 600; ++i)
    {
        double lons = lon.evalValue(t);
        double lonv = lon.evalDValue(t);
        double lats = leftlat.evalValue(t);
        double latv = leftlat.evalDValue(t);
        TrajectoryPoint p;
        p.t = t;
        p.x = lons;
        p.y = lats;
        p.v = sqrt(lonv*lonv+latv*latv);
        p.phi = atan2(latv,lonv);
        path.push_back(p);
        t += 0.01;
    }
    double tlast = t;
    double xlast = path[path.size()-1].x;
    double ylast = path[path.size()-1].y;
    t = 0.01;
    for(int i = 1; i < 600; ++i)
    {
        double lons = lon.evalValue(t);
        double lonv = lon.evalDValue(t);
        double lats = rightlat.evalValue(t);
        double latv = rightlat.evalDValue(t);
        TrajectoryPoint p;
        p.t = t + tlast;
        p.x = lons + xlast;
        p.y = lats + ylast;
        p.v = sqrt(lonv*lonv+latv*latv);
        p.phi = atan2(latv,lonv);
        path.push_back(p);
        t += 0.01;
    }
    path[0].Curvature = 0;
    path[path.size()-1].Curvature = 0;
    for(int i = 1; i < path.size()-1; ++i)
    {
        double ax = path[i-1].x;
        double bx = path[i  ].x;
        double cx = path[i+1].x;
        double ay = path[i-1].y;
        double by = path[i  ].y;
        double cy = path[i+1].y;
        path[i].Curvature = getCurvature(ax,ay,bx,by,cx,cy);
    }
}
TrajectoryPoint DesireTrajectory::QueryPathPointAtTime(double time)
{
    for(auto p: path)
    {
        if (fabs(time - p.t) < 0.001)
        {
            return p;
        }
    }
    return path[0];
}
