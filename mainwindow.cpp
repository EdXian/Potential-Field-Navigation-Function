#include "mainwindow.h"
#include "ui_mainwindow.h"


#define display_domain

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p(10,10,0.1),
    dataSpiral1(100)
{
    ui->setupUi(this);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    fermatSpiral1 = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    timer->start(50);
    ui->customPlot->xAxis->setRange(-10,10);
    ui->customPlot->yAxis->setRange(-10,10);
    //
#ifdef display_domain
    QCPAxisRect *   xRect = new QCPAxisRect( this->ui->customPlot );
    QCPItemRect *   xRectItem = new QCPItemRect( this->ui->customPlot );
    xRectItem->setVisible          (true);
    xRectItem->setPen              (QPen(Qt::blue));
    xRectItem->setBrush            (QBrush(Qt::transparent));
    xRectItem->topLeft     ->setType(QCPItemPosition::ptPlotCoords);
    xRectItem->topLeft     ->setAxisRect( xRect );
    xRectItem->topLeft     ->setCoords( -10, 10 );
    xRectItem->bottomRight ->setType(QCPItemPosition::ptPlotCoords);
    xRectItem->bottomRight ->setAxisRect( xRect );
    xRectItem->bottomRight ->setCoords( 10, -10 );
    xRectItem->setClipAxisRect     ( xRect );
    xRectItem->setClipToAxisRect   ( false );       // XXX
    ui->customPlot->replot();
#endif

    QVector<double> ox(400),oy(400);
//    for(int i=0;i<20;i++){
//        dot data;
//        for(int j=0;j<20;j++){
//            data.x = -3 + 0.1 *i;
//            data.y = -3 + 0.1 *j;
//            data.obstacle=true;
//            ox[i*20+j] = data.x;
//            oy[i*20+j] = data.y;
//            p.obstacle.push_back(data);
//        }
//    }
    ui->customPlot->addGraph();
    ui->customPlot->graph(graph_id)->setLineStyle(QCPGraph::LineStyle::lsNone);
    ui->customPlot->graph(graph_id)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui->customPlot->graph(graph_id)->setPen(QPen(Qt::green));
    ui->customPlot->graph(graph_id)->setData(ox,oy);
    graph_id++;
    //obstacle
 t.x =2; t.y =7;
 robot.x = -3.4;
 robot.y = -3.4;
 robot.radius = 0.5;
 robot.gain = 2.0;
}

void MainWindow::plot_loop(){

    for (int i=0; i<100; ++i)
    {
     dataSpiral1[i] = QCPCurveData(i, robot.x+robot.radius*cos(0.1*i), robot.y+robot.radius*sin(0.1*i));
    }
    fermatSpiral1->data()->set(dataSpiral1, true);

    p.detect_obstacle(robot, p.obstacle);
    p.gradient_phi(robot,t);
//    std::cout << robot.obstacle_detect.size()<<std::endl;

//    robot.x += 10000*robot.vx;
//    robot.y += 10000*robot.vy;

    std::cout << "rep = " <<robot.rep.x  <<"  "<<robot.rep.y<<std::endl;
    std::cout << "att = " <<robot.att.x  <<"  "<<robot.att.y<<std::endl;

    std::cout << "vx = " << robot.vx <<std::endl;
    std::cout << "vy = " << robot.vy <<std::endl;


    ui->customPlot->replot();
    robot.obstacle_detect.clear();

}
MainWindow::~MainWindow()
{
    delete ui;
}
