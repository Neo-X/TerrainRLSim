
#include <Eigen/Core>
#include <iostream>
#include "Rockstar.hpp"
#include "CMAES.hpp"

int main(){

  int n_paramter  = 20;
  int initial_exp = 2;

  Eigen::VectorXd initial_theta = Eigen::VectorXd::Zero(n_paramter,1);
  Eigen::VectorXd Initial_StandardDeviation = Eigen::VectorXd::Ones(n_paramter,1) * 0.05;
  Eigen::VectorXd theta = Eigen::VectorXd::Zero(n_paramter,1);

//  rockstar::Rockstar optimizer(initial_theta,Initial_StandardDeviation, initial_exp);
  cmaes::CMAES optimizer(initial_theta,Initial_StandardDeviation);

  for(int i=0; i<2000; i++){

    optimizer.getNextTheta2Evaluate(theta);
    double cost = rosen(theta);
//    std::cout<<theta<<std::endl<<", cost is "<<cost<<std::endl;

    optimizer.setTheCostFromTheLastTheta(cost);

    if(i % 30 == 0)
      std::cout<<optimizer.getRolloutNumber()<<"th rollout, this cost is   "<<cost<<", sigma is "<<optimizer.getSigma()<<std::endl;

    // we recommend to set a custom termination criteria!
    if(optimizer.isOptimizationDone())
      break;
  }

  return 0;
}


// rosenbrock function for testing
double rosen(Eigen::VectorXd x){

  //std::cout<<"x from evaluation function is :"<<std::endl<<x<<std::endl;
    double cost;
    int end = x.rows()-1;
    Eigen::VectorXd y = x.block(0,0,end,1);
    Eigen::VectorXd w = x.block(1,0,end,1);
    Eigen::VectorXd q(end);
    q.setOnes();
    q=y-q;

    cost = (y.cwiseProduct(y) - w).squaredNorm()*100.0 + q.squaredNorm();

  return cost;
}
