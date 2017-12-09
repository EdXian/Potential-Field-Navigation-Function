#include "potential_field.h"

potential_field::potential_field(float x_range, float y_range,double dq)
{
     dot data;

    for(int i=0 ; i<2*(x_range/dq); i++){

        for(int j=0 ; j<2*(y_range/dq) ; j++){
            data.x = -1*x_range + i*dq;
            data.y = -1*y_range + j*dq;
            this->q.push_back(data);
        }
    }

}


//compute navigation function
void potential_field::gradient_phi(agent& robot , dot& target){

    double gamma_ , beta_=1.0 , alpha;
    double  beta_l=1.0;
    vect rep_sum,rep;
    double h;

    rep.x = 1.0;
    rep.y = 1.0;
    gamma_ = this->gamma( robot , target);
    beta_ = this ->beta(robot);

    alpha = pow( gamma_ , robot.gain )+beta_;
    // -1 or 1
    alpha = 1/pow( alpha , (1/robot.gain + 1));

    robot.att.x = beta_ * 2* ( robot.x - target.x) ;
    robot.att.y = beta_ * 2* ( robot.y - target.y) ;

//    std::cout<< "-------------" <<std::endl;
//    std::cout <<"k = " << robot.gain  <<std::endl;
//    std::cout <<"alpha = " << alpha  <<std::endl;
//    std::cout <<"gamma_ = " << gamma_  <<std::endl;
//    std::cout <<"beta = " << beta_<<std::endl;
//    std::cout << "attx = " <<  robot.att.x
//              << "  atty = " <<   robot.att.y<<std::endl;


    if(robot.obstacle_detect.size()>1){
        for(unsigned int i=0;i<robot.obstacle_detect.size();i++){
            beta_l =1.0;
            h = zigma(robot ,robot.obstacle_detect[i]);
            rep.x =  h*gamma_ * robot.gain * (robot.x-robot.obstacle_detect[i].x);
            rep.y =  h*gamma_ * robot.gain * (robot.y-robot.obstacle_detect[i].y);

            for(unsigned int j =0 ;j<robot.obstacle_detect.size();j++){
                if(j!=i){

                    beta_l  *=  sigmod(robot , robot.obstacle_detect[j]);

                }

            }
            rep_sum.x += rep.x * beta_l;
            rep_sum.y += rep.y * beta_l;
        }
        robot.rep.x = rep_sum.x;
        robot.rep.y = rep_sum.y;

    }else if(robot.obstacle_detect.size()==1){

        h = zigma(robot ,robot.obstacle_detect[0]);
        rep.x =  h*gamma_  * (robot.x-robot.obstacle_detect[0].x);
        rep.y =  h*gamma_  * (robot.y-robot.obstacle_detect[0].y);
        robot.rep.x = rep.x * beta_l;
        robot.rep.y = rep.y * beta_l;
    }

   robot.vx = -1*alpha * (robot.att.x - robot.rep.x) ;
   robot.vy = -1*alpha * (robot.att.y - robot.rep.y) ;
}

//detect obstacles ok
void potential_field::detect_obstacle(agent& robot , std::vector<dot>& obstacle){
   dot data;
    for(std::vector<dot>::iterator it =obstacle.begin() ; it!= obstacle.end();it++){
            data.x = (*it).x ;
            data.y = (*it).y ;
            robot.obstacle_detect.push_back(data);
    }
  //  std::cout << "size = "<< robot.obstacle_detect.size() << std::endl;
}

// goal function. ok
double potential_field::gamma(agent& robot , dot& target){
    return (robot.x - target.x)*(robot.x - target.x) + (robot.y - target.y) * (robot.y - target.y);
}

double potential_field::zigma(agent robot ,dot obstacle){
    double  value1 ,value2 ,value3;
    double h ;
    dot data;
    data.x = robot.x;
    data.y = robot.y;

    h = ((robot.radius/2.0)-distance(data ,obstacle )); //*(12/robot.radius)
    value1 = 1/((1+exp(h))*(1+exp(h)));
    value2 = exp(h)/robot.radius;
    value3 = 1/ distance(data , obstacle);

    return value1 * value2 * value3;
}

// Obstacle avoidance function ok
double potential_field::beta(agent robot){
    double beta_i = 1.0, beta_il;
    if(robot.obstacle_detect.size()>1){
        for(unsigned int i=0;i<robot.obstacle_detect.size();i++){

            beta_il = sigmod(robot,robot.obstacle_detect[i]);
            beta_i = beta_i * beta_il;
        }
    }
    if(robot.obstacle_detect.size()==1){
      beta_i = sigmod( robot ,robot.obstacle_detect[0]);
      std::cout << "beta_i"  <<  beta_i<<std::endl;
    }
    //if p_il >robot.radius  beta_i =1;

    return beta_i;
}

// sigmod function ok
double potential_field::sigmod(agent robot ,dot obstacle){
    double value=0.0;
    double dist ;
    dot data;
    data.x = robot.x;
    data.y = robot.y;
    dist = distance(data , obstacle);
    value = 1 + exp(-1*(dist - (robot.radius/2))*(1)) ; //   3/robot.radius
    value = 1/value;
    //std::cout << ""  <<  value<<std::endl;
    return value;
}
void phi(agent& robot , dot& target){




}

// return distance between a and b ok
double potential_field::distance(dot a,dot b){
    return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
}
