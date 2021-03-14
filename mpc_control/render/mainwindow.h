#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QWheelEvent>
#include <QPainter>
#include "planning/DesireTrajectory.h"
#include "vehiclemodel/Vehicle.h"
#include "vehcontroller/MPCControler.h"
struct Axes{
    double xfac;
    double yfac;
    double ymax;
    Axes(double xf, double yf, double ymax_):
            xfac(xf),yfac(yf),ymax(ymax_)
    {
    }
    void zoomOut(){
        xfac -= 0.1;
        yfac -= 0.1;
    };
    void zoomIn(){
        xfac += 0.1;
        yfac += 0.1;
    };
};
class QAxesPainter
{
private:
    QPainter *painter;
    Axes *axes;
public:
    QAxesPainter(QPainter* painter_, Axes* axes_)
            :painter(painter_),axes(axes_)
    {
    }
    void drawLine(double x1, double y1, double x2, double y2)
    {
        int X1 = axes->xfac*x1;
        int Y1 = axes->ymax - axes->yfac*y1;
        int X2 = axes->xfac*x2;
        int Y2 = axes->ymax - axes->yfac*y2;
        painter->drawLine(X1, Y1,X2,Y2);
    }
};
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
protected:
    void paintEvent(QPaintEvent *e) override ;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void mousePressEvent(QMouseEvent *ev) override;
    void mouseMoveEvent(QMouseEvent *ev) override;
    void wheelEvent(QWheelEvent*event) override;
private slots:
    void onUpdate();
private:
    QTimer *updateTimer = nullptr;
    bool dragFlag = false;
    int translateX = 0, translateY = -500;
    int dragStartX = 0, dragStartY = 0;
    int dragEndX = 0, dragEndY = 0;
    int oldX = 0, oldY = 0;
    Axes axes{40.0, 40.0, 1000};
private:
    DesireTrajectory traj;
    Vehicle veh;
    MPCController controller;
    VehicleState vstate;
    ControlCommand cmd;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> phis;
    double dT = 0.01;
    double t = 0.0;
};
#endif // MAINWINDOW_H
