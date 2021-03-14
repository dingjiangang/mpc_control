#include "mainwindow.h"
#include <QPainter>
#include <QMouseEvent>
#include <iostream>
#include <memory>
#include <QGraphicsView>
#include <QGraphicsPathItem>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    updateTimer = new QTimer(this);
    updateTimer->setInterval(50);
    updateTimer->start();
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(onUpdate()));

    traj.SetDemoTrajData();
    veh.setSimDt(dT);
    veh.setInitPose(0, 0, 0);
    veh.setInitSpeed(10.0);
}
void MainWindow::onUpdate()
{
    if(t >12.0)
    {
        update();
        return;
    }
    // Get Vehicle State
    vstate.xpos = veh.getX();
    vstate.ypos = veh.getY();
    vstate.speed = veh.getSpeed();
    vstate.phi = veh.getHeading();
    vstate.dphi = veh.getHeadingRate();
    // Compute control command
    controller.ComputeControlCommand(t, &vstate, &traj, &cmd);
    // Save data
    xs.push_back(vstate.xpos);
    ys.push_back(vstate.ypos);
    phis.push_back(vstate.phi);
    // Simulate
    veh.setAcc(cmd.acc);
    veh.setSteer(cmd.steer);
    veh.update();
    // update time
    t += dT;
    update();
}
void MainWindow::paintEvent(QPaintEvent *e)
{
    QPainter p(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    QPixmap pix;
    pix.load("/home/naruto/mpc_control-master/mpc_control/vehicle.png");
    int w = 1000;
    int h = 1000;
    p.setWindow(0, 0, w, h);
    p.translate(translateX,translateY);
    p.translate(-vstate.xpos * axes.xfac +400, 0);
    QAxesPainter pa(&p, &axes);

    //draw axes
    int dx = -p.transform().dx();
    int dy = -p.transform().dy();
    int orginX = 100;
    int orginY = 900;
    double stepX = w/10.0;
    double stepY = h/10.0;
    p.setPen(Qt::darkGray);
    p.drawLine(0+dx,orginY+dy,w+dx,orginY+dy);
    p.drawLine(orginX+dx,0+dy,orginX+dx,h+dy);
    int lineW = 5;
    p.setPen(Qt::gray);
    for(int i = 0; i < 10; ++i)
    {
        p.drawLine(orginX+dx-lineW,0+i*stepY+dy,orginX+dx+lineW,0+i*stepY+dy);
        double y = (axes.ymax - i*stepY - dy)/axes.yfac;
        p.drawText(orginX+dx,0+i*stepY+dy,QString::number(y));
        p.drawLine(0+i*stepX+dx,orginY+dy-lineW,0+i*stepX+dx,orginY+dy+lineW);
        double x = (i*stepX+dx)/axes.xfac;
        p.drawText(0+i*stepX+dx,orginY+dy+4*lineW,QString::number(x));
    }
    //draw planning trajectory
    p.setPen(Qt::blue);
    for(int i = 0; i < traj.path.size()-1; ++i)
    {
        pa.drawLine(traj.path[i].x,traj.path[i].y,traj.path[i+1].x,traj.path[i+1].y);
    }

    p.setPen(Qt::black);
    //draw road
    pa.drawLine(0, -2, 120, -2);
    pa.drawLine(0, 2, 120, 2);
    pa.drawLine(0, 6, 120, 6);
    p.setPen(Qt::red);
    //draw actual trajectory
    for(int i = 1; i < xs.size() && i< ys.size(); ++i)
    {
        pa.drawLine(xs[i-1],ys[i-1],xs[i],ys[i]);
    }

    //draw vehicle
    QMatrix matrix;
    matrix.scale(1.8*axes.xfac/pix.width(),4.0*axes.yfac/pix.height());
    matrix.rotate(-vstate.phi*180.0/M_PI-90.0);
    pix = pix.transformed(matrix, Qt::FastTransformation);
    p.drawPixmap(vstate.xpos*axes.xfac-pix.width()/2.0,axes.ymax-vstate.ypos*axes.xfac-pix.height()/2.0,pix.width(),pix.height(),pix);
/*
    double H = 2.0,W = 1.0;
    QPainterPath path;
    double x1 = vstate.xpos + cos(vstate.phi)*H - sin(vstate.phi)*W;
    double y1 = vstate.ypos + sin(vstate.phi)*H + cos(vstate.phi)*W;

    double x2 = vstate.xpos + cos(vstate.phi)*H + sin(vstate.phi)*W;
    double y2 = vstate.ypos + sin(vstate.phi)*H - cos(vstate.phi)*W;

    double x3 = vstate.xpos - cos(vstate.phi)*H - sin(vstate.phi)*W;
    double y3 = vstate.ypos - sin(vstate.phi)*H + cos(vstate.phi)*W;

    double x4 = vstate.xpos - cos(vstate.phi)*H + sin(vstate.phi)*W;
    double y4 = vstate.ypos - sin(vstate.phi)*H - cos(vstate.phi)*W;
    path.lineTo(x1*axes.xfac, axes.ymax-y1*axes.yfac);
    path.lineTo(x2*axes.xfac, axes.ymax-y2*axes.yfac);
    path.lineTo(x4*axes.xfac, axes.ymax-y4*axes.yfac);
    path.lineTo(x3*axes.xfac, axes.ymax-y3*axes.yfac);
    path.lineTo(x1*axes.xfac, axes.ymax-y1*axes.yfac);
    p.fillPath(path, Qt::blue);
    */
    p.setPen(Qt::green);
    p.drawText(vstate.xpos*axes.xfac, axes.ymax-vstate.ypos*axes.xfac, QString::number(vstate.xpos)+","+QString::number(vstate.ypos));
}
void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
    dragFlag = false;
}
void MainWindow::mousePressEvent(QMouseEvent *ev)
{
    if(ev->button() == Qt::RightButton)
    {
        dragFlag = true;
        dragStartX = ev->x();
        dragStartY = ev->y();
        oldX = translateX;
        oldY = translateY;
    }
}
void MainWindow::mouseMoveEvent(QMouseEvent *ev)
{
    if(dragFlag)
    {
        dragEndX = ev->x();
        dragEndY = ev->y();
        int dx = dragEndX - dragStartX;
        int dy = dragEndY - dragStartY;
        translateX = (oldX + dx);
        translateY = (oldY + dy);
    }
}
void MainWindow::wheelEvent(QWheelEvent*event){
    if(event->delta()>0){//如果滚轮往上滚
        axes.zoomIn();
    }else{//同样的
        axes.zoomOut();
    }
}
MainWindow::~MainWindow()
{
}
