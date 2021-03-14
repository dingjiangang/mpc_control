#include "OpenGLPlot.h"
#include <cstdlib>
struct Vertex
{
    GLfloat x,y,z;
    GLfloat r,g,b,a;
    Vertex(GLfloat x_, GLfloat y_, GLfloat z_,GLfloat r_,GLfloat g_,GLfloat b_,GLfloat a_)
    {
        x = x_;
        y = y_;
        z = z_;
        r = r_;
        g = g_;
        b = b_;
        a = a_;
    }
    Vertex(GLfloat x_, GLfloat y_, GLfloat z_)
    {
        x = x_;
        y = y_;
        z = z_;
        r = 1.0f;
        g = 1.0f;
        b = 1.0f;
        a = 1.0f;
    }
};
void drawLineSegment(Vertex v1, Vertex v2, GLfloat width)
{
    glLineWidth(width);
    glBegin(GL_LINES);
    glColor4f(v1.r, v1.g, v1.b, v1.a);
    glVertex3f(v1.x, v1.y, v1.z);
    glColor4f(v2.r, v2.g, v2.b, v2.a);
    glVertex3f(v2.x, v2.y, v2.z);
    glEnd();
}
void Curve2D::Plot()
{
    for(int i = 0; i < x.size()-1; ++i)
    {
        GLfloat x1 = x[i];
        GLfloat y1 = y[i];
        GLfloat x2 = x[i+1];
        GLfloat y2 = y[i+1];
        Vertex v1{x1,y1,0.f,r,g,b,1.0f};
        Vertex v2{x2,y2,0.f,r,g,b,1.0f};
        drawLineSegment(v1,v2,4.0f);
    }
}

OpenGLPlot::OpenGLPlot()
{

    if(!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OpenGLPlot", NULL, NULL);
    if(!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    std::vector<double> xAxisxData;
    std::vector<double> xAxisyData;
    std::vector<double> yAxisxData;
    std::vector<double> yAxisyData;
    double x = 0.0;
    double y = 0.0;
    double xStep = 0.1;
    double yStep = 0.02;
    for(int i = 0; i < 100; ++i)
    {
        xAxisxData.push_back(x);
        xAxisyData.push_back(0.0);
        yAxisxData.push_back(0.0);
        yAxisyData.push_back(y);
        y += yStep;
        x += xStep;
    }
    AddCurve(Curve2D(xAxisxData, xAxisyData));
    AddCurve(Curve2D(yAxisxData, yAxisyData));
}
void OpenGLPlot::MainLoop()
{
    while(!glfwWindowShouldClose(window))
    {
        float ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = (float)width/(float)height;
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-1.f, 10.f, -1.f, 2.f, 1.f, -1.f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        PlotCurves();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
OpenGLPlot::~OpenGLPlot()
{
    glfwDestroyWindow(window);
    glfwTerminate();
}
