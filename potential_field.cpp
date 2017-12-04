#include "potential_field.h"

potential_field::potential_field(float x_range, float y_range,double dq)
{
     dot data;

    for(int i=0 ; i<2*(x_range/dq); i++){

        for(int j=0 ; j<2*(y_range/dq) ; j++){
            data.x = -1*x_range + i*dq;
            data.y = -1*y_range + j*dq;
            //std::cout<< "x = " << data.x <<"y = "<<data.y<<std::endl;
            this->q.push_back(data);
        }
    }

}
void potential_field::detect_obstacle(){


}

void potential_field::gradient_phi(agent& robot , dot& target){

    double gamma_ , beta_ , alpha;
    double dt=0.01 ;
    gamma_ = this->gamma( robot , target);

    beta_ = this ->beta(robot);
    robot.beta = beta_;

    alpha = pow( gamma_ , robot.gain )+beta_;
    alpha = pow( alpha , -1*(robot.gain + 1));





    robot.x +=  dt * (robot.vx);
    robot.y +=  dt * (robot.vy);
    robot.beta_last =robot.beta;
}

// goal function.
double potential_field::gamma(agent& robot , dot& target){
    return (robot.x - target.x)*(robot.x - target.x) + (robot.y - target.y) * (robot.y - target.y);
}
// Obstacle avoidance function
double potential_field::beta(agent& robot){
    double p_il;
    double beta_i = 1.0, beta_il;
    for(std::vector<dot>::iterator it = robot.obstacle.begin() ; it!=robot.obstacle.end();it++)
    {
        p_il = (robot.x - (*it).x) * (robot.x - (*it).x) + (robot.y - (*it).y) * (robot.y - (*it).y);
        p_il = sqrt(p_il);
        beta_il = 1/(1+exp(-1*(p_il-robot.radius)*(12/robot.radius)));
        beta_i *= beta_il;
    }
    return beta_i;
}

