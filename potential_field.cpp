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

//detect obstacles
void potential_field::detect_obstacle(agent& robot , std::vector<dot>& obstacle){
    for(std::vector<dot>::iterator it =obstacle.begin() ; it!= obstacle.end();it++){
        dot data;
        double  distance =sqrt( (robot.x-(*it).x )*(robot.x-(*it).x )+(robot.y-(*it).y )*(robot.y-(*it).y) );
        if(distance < robot.radius){
            data.x = (*it).x ;
            data.y = (*it).y ;
            robot.obstacle_detect.push_back(data);

        }
    }
  //  std::cout << "size = "<< robot.obstacle_detect.size() << std::endl;
}


//compute navigation function
void potential_field::gradient_phi(agent& robot , dot& target){

    double gamma_ , beta_ , alpha;
    double dt=0.01 ;

    double att_x = 0.0 , att_y = 0.0;
    double rep_x = 0.0 , rep_y = 0.0;

    gamma_ = this->gamma( robot , target);
    beta_ = this ->beta(robot);
    robot.beta = beta_;

    alpha = pow( gamma_ , robot.gain )+beta_;
    alpha = 1*pow( alpha , -1*(robot.gain + 1));

    //calc vel
    robot.vx = robot.beta * 2* (robot.x - target.x) -
               (gamma_ / robot.gain)* ((robot.beta_last -robot.beta)/(robot.x_last - robot.x));

    robot.vy = robot.beta * 2* (robot.y - target.y) -
               (gamma_ / robot.gain)* ((robot.beta_last -robot.beta)/(robot.y_last - robot.y));

    //the dynamics of robots
//    robot.x +=  dt * (robot.vx);
//    robot.y +=  dt * (robot.vy);
}


// goal function.
double potential_field::gamma(agent& robot , dot& target){
    return (robot.x - target.x)*(robot.x - target.x) + (robot.y - target.y) * (robot.y - target.y);
}


// Obstacle avoidance function
double potential_field::beta(agent& robot){
    double p_il;

    double beta_i = 1.0, beta_il;

    if(robot.obstacle_detect.size()>0){
        for(int i=0;i<robot.obstacle_detect.size();i++){

            for(int j=0;j<robot.obstacle_detect.size();j++){
                if(j!=i){


                    //compute pi function
                }
            }

        }
    }
    //if p_il >robot.radius  beta_i =1;
    return beta_i;
}

double potential_field::pi(){

}

double potential_field::distance(dot a,dot b){
    return sqrt(  (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
}
