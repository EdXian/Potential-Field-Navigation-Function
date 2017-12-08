#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "stdlib.h"
#include "stdio.h"
#include "qcustomplot.h"
#include "iostream"
#include "potential_field.h"
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QVector<double> q;
    agent robot;
    dot t;
    QTimer *timer;
    int graph_id=0;
   // potential_field p(10,10,0.1);
   // potential_field p(10,10,0.1);
    potential_field p;
    QVector<QCPCurveData> robot_data;
    QCPCurve *robot_curve;

    QVector<QCPCurveData> obstacle_data;
    QCPCurve *obstacle_curve;

    QVector<double> rl_t,rl_y,rl_z;
    void vel_plot(double x,double y);
    double count;
private slots:

   void plot_loop();
private:
  //  void define_domain(float x_range, float y_range,double dq,QVector<double>& q);

    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
