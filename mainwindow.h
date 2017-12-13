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
    dot target;
    QTimer *timer;
    int graph_id=0;

    potential_field p;
    //robot
    QVector<QCPCurveData> robot_data;
    QCPCurve *robot_curve;

    //obstacle
    QVector<QCPCurveData> obstacle_data;
    QCPCurve *obstacle_curve;

    QVector<QCPCurveData> obstacle_data1;
    QCPCurve *obstacle_curve1;

    QVector<QCPCurveData> obstacle_data2;
    QCPCurve *obstacle_curve2;

    QVector<double> rl_t,rl_y,rl_z;

    //color map
    QCPColorMap *colorMap;
    QCPColorScale *colorScale;
    QCPMarginGroup *marginGroup;
    void vel_plot(double x,double y);
    double count;

    dot obstacle_pos;

private slots:

   void plot_loop();
   void on_pushButton_clicked();
   void on_pushButton_2_clicked();

   void on_pushButton_3_clicked();

   void on_checkBox_clicked();

private:
  //  void define_domain(float x_range, float y_range,double dq,QVector<double>& q);

    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
