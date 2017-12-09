#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H
#include "stdio.h"
#include "iostream"
#include "vector"
#include "math.h"
using namespace std;

struct vect
{
    double x;
    double y;
};

struct dot{
  double x;
  double y;
  double value;

};
typedef vect velocity;
typedef dot position;
struct agent{
    position pos;
    velocity vel;
    vect att; // attractive force
    vect rep; // repulsive force
    std::vector<dot> obstacle_detect;
    double radius;
    double beta;
    double gain;  // 0<k<1
};




class potential_field
{
public:

    potential_field(float x_range, float y_range,double dq);
    std::vector<dot> q;
    std::vector<dot> obstacle;
     double distance(dot a,dot b);
    void detect_obstacle(agent& robot , std::vector<dot>& obstacle);
    double gamma(agent& robot , dot& target);
    double beta(agent robot);

    void gradient_phi(agent& robot , dot& target);
    double phi(double x ,double y,agent robot , dot target);
    double sigmod(agent robot ,dot obstacle);

    double zigma(agent robot ,dot obstacle);
private:

};

#endif // POTENTIAL_FIELD_H
