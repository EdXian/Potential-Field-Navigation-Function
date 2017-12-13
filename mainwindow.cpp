#include "mainwindow.h"
#include "ui_mainwindow.h"


//#define display_domain
//#define colormap
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


    //add obstacle

    obstacle_pos.x = 5;
    obstacle_pos.y = 5;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve2->setPen(QPen(Qt::red));
    obstacle_curve2->data()->set(robot_data, true);

    obstacle_pos.x = 4.5;
    obstacle_pos.y = -3;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve1->setPen(QPen(Qt::red));
    obstacle_curve1->data()->set(robot_data, true);

    obstacle_pos.x = 4;
    obstacle_pos.y = 1;
    p.obstacle.push_back(obstacle_pos);
    for (int i=0; i<100; ++i)
    {
      robot_data[i] = QCPCurveData(i, obstacle_pos.x+0.8*cos(0.1*i), obstacle_pos.y+0.8*sin(0.1*i));
    }
    obstacle_curve->setPen(QPen(Qt::red));
    obstacle_curve->data()->set(robot_data, true);

    target.x =12; target.y =11;
    //robot initial pose
    robot.pos.x = -2;  //-3.35
    robot.pos.y = -7;  //-3.35
    robot.radius = 1.0;
    robot.gain = 0.8;

#ifdef colormap
    colorMap = new QCPColorMap(ui->customPlot->xAxis, ui->customPlot->yAxis);
    colorScale = new QCPColorScale(ui->customPlot);
    marginGroup = new QCPMarginGroup(ui->customPlot);
    p.detect_obstacle(robot , p.obstacle);
    int nx = 200;
    int ny = 200;
    colorMap->data()->setSize(nx, ny);
    colorMap->data()->setRange(QCPRange(-10, 10), QCPRange(-10, 10));
    double x, y, z,value;
    double dist;
    agent r;
    for (int xIndex=0; xIndex<nx; ++xIndex)
    {
        for (int yIndex=0; yIndex<ny; ++yIndex)
        {
            colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
            //z = p.phi(x,y,robot,t);

            r.pos.x = x;
            r.pos.y = y;

            dist = p.gamma(r,t);
            double b=1.0;
            for(int i=0;i<robot.obstacle_detect.size();i++)
            {
                b *= p.sigmod(r,robot.obstacle_detect[i]);
            }
            value = pow(dist , robot.gain)+b;
            value = pow(value,(1/robot.gain));
            z = dist / value ;
            colorMap->data()->setCell(xIndex, yIndex, z);
        }
    }
    ui->customPlot->plotLayout()->addElement(0, 1, colorScale);
    colorScale->setType(QCPAxis::atRight);
    colorMap->setColorScale(colorScale);
    colorScale->axis()->setLabel("Potential Field");
    colorMap->setGradient(QCPColorGradient::gpJet);
    colorMap->rescaleDataRange();
    ui->customPlot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
    colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
    ui->customPlot->rescaleAxes();
    robot.obstacle_detect.clear();
#endif

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this, SLOT(plot_loop()));
    timer->start(50);
}

void MainWindow::plot_loop(){


    //plot robot
    for (int i=0; i<100; ++i)
    {
     robot_data[i] = QCPCurveData(i, robot.pos.x+robot.radius*cos(0.1*i), robot.pos.y+robot.radius*sin(0.1*i));
    }
    robot_curve->setPen(QPen(Qt::blue));
    robot_curve->data()->set(robot_data, true);

    //
    p.detect_obstacle(robot , p.obstacle);
    p.gradient_phi(robot,target);

    robot.vel.x *= 40;
    robot.vel.y *=40;

    if(robot.vel.x > 0.3){
        robot.vel.x=0.3;
    }else if(robot.vel.x <-0.3){
        robot.vel.x=-0.3;
    }
    if(robot.vel.y > 0.3){
        robot.vel.y=0.3;
    }else if(robot.vel.y <-0.3){
        robot.vel.y=-0.3;
    }
    //vel_plot(robot.vel.x,robot.vel.y);

    if (p.distance(robot.pos , target)<1){
        robot.pos.x += -0.1*(robot.pos.x - target.x);
        robot.pos.y += -0.1*(robot.pos.y - target.y);

    }else{
        robot.pos.x +=robot.vel.x;
        robot.pos.y +=robot.vel.y;
    }

    ui->customPlot->replot();
    ui->realtime_plot->replot();
    robot.obstacle_detect.clear();
}

void MainWindow::vel_plot(double x, double y){
    rl_t.push_back(count*0.1);
    rl_y.push_back(20*x);
    rl_z.push_back(20*y);
    if(rl_t.size()>50)
    {
        ui->realtime_plot->xAxis->setRange(-5+count*0.1,5+count*0.1);
        ui->realtime_plot->yAxis->setRange(-10,10);
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

void MainWindow::on_pushButton_3_clicked()
{
    target.x = ui->target_x->text().toDouble();
    target.y = ui->target_y->text().toDouble();
}

void MainWindow::on_checkBox_clicked()
{

    if(ui->checkBox->isChecked()){
        std::cout << "clicked" <<std::endl;
        colorMap = new QCPColorMap(ui->customPlot->xAxis, ui->customPlot->yAxis);
        colorScale = new QCPColorScale(ui->customPlot);
        marginGroup = new QCPMarginGroup(ui->customPlot);
        p.detect_obstacle(robot , p.obstacle);
        int nx = 200;
        int ny = 200;
        colorMap->data()->setSize(nx, ny);
        colorMap->data()->setRange(QCPRange(-10, 10), QCPRange(-10, 10));
        double x, y, z,value;
        double dist;
        agent r;
        for (int xIndex=0; xIndex<nx; ++xIndex)
        {
            for (int yIndex=0; yIndex<ny; ++yIndex)
            {
                colorMap->data()->cellToCoord(xIndex, yIndex, &x, &y);
                //z = p.phi(x,y,robot,t);

                r.pos.x = x;
                r.pos.y = y;

                dist = p.gamma(r,target);
                double b=1.0;
                for(int i=0;i<robot.obstacle_detect.size();i++)
                {
                    b *= p.sigmod(r,robot.obstacle_detect[i]);
                }
                value = pow(dist , robot.gain)+b;
                value = pow(value,(1/robot.gain));
                z = dist / value ;
                colorMap->data()->setCell(xIndex, yIndex, z);
            }
        }

//        ui->customPlot->plotLayout()->addElement(0, 1, colorScale);
        colorScale->setType(QCPAxis::atRight);
        colorMap->setColorScale(colorScale);

        colorMap->setGradient(QCPColorGradient::gpJet);
        colorMap->rescaleDataRange();
        ui->customPlot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
        colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
        ui->customPlot->rescaleAxes();
        robot.obstacle_detect.clear();

    }else{
         colorMap->data()->clear();

    }
}
