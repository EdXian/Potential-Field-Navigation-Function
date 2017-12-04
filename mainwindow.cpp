#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "algorithm"
#include <utility>
#include <boost/graph/graph_traits.hpp>
 #include <boost/graph/adjacency_list.hpp>
 #include <boost/graph/dijkstra_shortest_paths.hpp>
#define display_domain

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


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


    potential_field p(10,10,0.1);
    float d=0.1;


    QVector<double> x(100) , y(100);
    for(int i =0; i<10.0/d;i++)
    {
            x[i]=i*0.1;
            y[i] =1/(1+ exp(x[i]));
    }



    ui->customPlot->xAxis->setRange(-10,10);
    ui->customPlot->yAxis->setRange(-10,10);
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setData(x,y);
    ui->customPlot->replot();

//    QCPCurve *fermatSpiral1 = new QCPCurve(ui->customPlot->xAxis, ui->customPlot->yAxis);
//    QVector<QCPCurveData> dataSpiral1(100);
//    int a=3 ,b=4;
//    for (int i=0; i<100; ++i)
//    {
//      dataSpiral1[i] = QCPCurveData(i, a*cos(0.1*i), b*sin(0.1*i));
//    }
//    fermatSpiral1->data()->set(dataSpiral1, true);

}

MainWindow::~MainWindow()
{
    delete ui;
}
