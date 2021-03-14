#ifndef MPC_CONTROL_OPENGLPLOT_H
#define MPC_CONTROL_OPENGLPLOT_H

#include <GLFW/glfw3.h>
#include <vector>
class Curve2D
{
private:
    std::vector<double> x;
    std::vector<double> y;
    GLfloat r=1.f,g=1.f,b=1.f;
public:
    Curve2D(const std::vector<double>& x_, const std::vector<double>& y_,double r_,double g_,double b_)
    {
        x = x_;
        y = y_;
        r = r_;
        g = g_;
        b = b_;
    }
    Curve2D(const std::vector<double>& x_, const std::vector<double>& y_)
    {
        x = x_;
        y = y_;
    }
public:
    void Plot();
};



class OpenGLPlot {
private:
    const int WINDOW_WIDTH = 640*2;
    const int WINDOW_HEIGHT = 480;
    GLFWwindow* window = nullptr;
    std::vector<Curve2D> curveList;
public:
    OpenGLPlot();
    void MainLoop();
    ~OpenGLPlot();
    void AddCurve(const Curve2D& curve)
    {
        curveList.push_back(curve);
    }
private:
    void PlotCurves()
    {
        for(auto c : curveList)
            c.Plot();
    }
};


#endif //MPC_CONTROL_OPENGLPLOT_H
