#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H
#include "stdio.h"
#include "iostream"
#include "vector"
#include "math.h"
using namespace std;



struct dot{
  double x;
  double y;
  double value;
  bool obstacle;
};


struct agent{
    double x;
    double y;
    double x_last;
    double y_last;
    double vx;
    double vy;
    std::vector<dot> obstacle_detect;
    double radius;
    double beta_last;
    double beta;
    double gain;
};




class potential_field
{
public:

    potential_field(float x_range, float y_range,double dq);
    std::vector<dot> q;
    std::vector<dot> obstacle;
    void detect_obstacle(agent& robot , std::vector<dot>& obstacle);
    double gamma(agent& robot , dot& target);
    double beta(agent& robot);
    double distance(dot a,dot b);
    void gradient_phi(agent& robot , dot& target);
    double pi();


private:

};

#endif // POTENTIAL_FIELD_H
