#include "mainwindow.h"
#include "ui_mainwindow.h"


//#define display_domain

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p(10,10,0.1),
    robot_data(100)
{
    ui->setupUi(this);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->realtime_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    robot_curve = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);

    obstacle_curve = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);
    obstacle_curve1 = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);
    obstacle_curve2 = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);
    ui->customPlot->xAxis->setRange(-10,10);
    ui->customPlot->yAxis->setRange(-10,10);

    ui->realtime_plot->yAxis->setRange(-5,5);
    ui->realtime_plot->xAxis->setLabel("time");
    ui->realtime_plot->yAxis->setLabel("vel");
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

    dot obstacle_pos;
    obstacle_pos.x = 1;
    obstacle_pos.y = 0;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve2->setPen(QPen(Qt::red));
    obstacle_curve2->data()->set(robot_data, true);

    obstacle_pos.x = 6;
    obstacle_pos.y = 0;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve1->setPen(QPen(Qt::red));
    obstacle_curve1->data()->set(robot_data, true);

    obstacle_pos.x = 6;
    obstacle_pos.y = 5;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve->setPen(QPen(Qt::red));
    obstacle_curve->data()->set(robot_data, true);

    t.x =9; t.y =9;
    robot.x = -2;  //-3.35
    robot.y = -7;  //-3.35
    robot.radius = 1.0;
    robot.gain = 0.8;





    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    timer->start(50);
}

void MainWindow::plot_loop(){


    //plot robot
    for (int i=0; i<100; ++i)
    {
     robot_data[i] = QCPCurveData(i, robot.x+robot.radius*cos(0.1*i), robot.y+robot.radius*sin(0.1*i));
    }
    robot_curve->setPen(QPen(Qt::blue));
    robot_curve->data()->set(robot_data, true);

    //
    p.detect_obstacle(robot , p.obstacle);
    p.gradient_phi(robot,t);

    robot.vx *= 40;
    robot.vy *=40;

    if(robot.vx > 0.3){
        robot.vx=0.3;
    }else if(robot.vx <-0.3){
        robot.vx=-0.3;
    }
    if(robot.vy > 0.3){
        robot.vy=0.3;
    }else if(robot.vy <-0.3){
        robot.vy=-0.3;
    }
    vel_plot(robot.vx,robot.vy);
    dot r;
    r.x = robot.x;
    r.y = robot.y;
    if (p.distance(r , t)<1){
        robot.x += -0.1*(robot.x - t.x);
        robot.y += -0.1*(robot.y - t.y);
        std::cout<<"ok"<<std::endl;
    }else{
        robot.x +=robot.vx;
        robot.y +=robot.vy;
    }








    ui->customPlot->replot();
    ui->realtime_plot->replot();
    robot.obstacle_detect.clear();
}

void MainWindow::vel_plot(double x, double y){
    rl_t.push_back(count*0.1);
    rl_y.push_back(x);
    rl_z.push_back(y);
    if(rl_t.size()>50)
    {
        ui->realtime_plot->xAxis->setRange(-5+count*0.1,5+count*0.1);
        ui->realtime_plot->yAxis->setRange(y-40,y+40);
        rl_t.pop_front();
        rl_y.pop_front();
        rl_z.pop_front();
    }
    ui->realtime_plot->addGraph();
    ui->realtime_plot->graph(0)->setPen(QPen(Qt::blue));
    ui->realtime_plot->graph(0)->setData(rl_t,rl_y);
    ui->realtime_plot->addGraph();
    ui->realtime_plot->graph(1)->setPen(QPen(Qt::red));
    ui->realtime_plot->graph(1)->setData(rl_t,rl_z);
    count++;
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    if(timer->isActive()){
        timer->stop();
    }else{
        timer->start();
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    QString fileName = "/velocity.jpg";
    QString outputDir = "~/Desktop";
    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << file.errorString();
    } else {
        //ui->realtime_plot->grab().save(outputDir+fileName);
        //ui->realtime_plot->savePng(outputDir+fileName);

        ui->realtime_plot->saveJpg( outputDir+fileName,  480, 400, 1.0, -1  );
        std::cout <<"Image is saved"<<std::endl;

    }


}
