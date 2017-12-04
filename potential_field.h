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
    double vx;
    double vy;
    std::vector<dot> obstacle;
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
    void detect_obstacle();
    double gamma(agent& robot , dot& target);
    double beta(agent& robot);
    void gradient_phi(agent& robot , dot& target);

private:

};

#endif // POTENTIAL_FIELD_H
