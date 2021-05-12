//%
//%
//% Copyright (c) 2014, ADRL/ETHZ. Jemin Hwangbo
//% All rights reserved.
//%
//% Redistribution and use in source and binary forms, with or without
//% modification, are permitted provided that the following conditions are met:
//%     * Redistributions of source code must retain the above copyright
//%       notice, this list of conditions and the following disclaimer.
//%     * Redistributions in binary form must reproduce the above copyright
//%       notice, this list of conditions and the following disclaimer in the
//%       documentation and/or other materials provided with the distribution.
//%     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
//%       names of its contributors may be used to endorse or promote products
//%       derived from this software without specific prior written permission.
//%
//% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//% DISCLAIMED. IN NO EVENT SHALL Jemin Hwangbo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
//% OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//% GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//% HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//% OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//%

/*!
* @file   Rockstar.hpp
* @author   Jemin Hwangbo
* @date   March, 2015
* @version  1.0
* @brief  Rock* - Efficient Black-Box Policy Optimization
 */
#ifndef ROCKSTAR_HPP_
#define ROCKSTAR_HPP_

#include <chrono>
#include <math.h>
#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <gsl/gsl_cdf.h> // inverse of Chi-squared cumulative distribution function

namespace rockstar {

//! Rock* algorithm
/*! Reward Optimization with compact kernels and natural gradient descent, 2015
 * Jemin Hwangbo
 */

class Rockstar {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! column vector with the size equal to the number of policy parameters
  typedef Eigen::VectorXd VectorNd;
  typedef Eigen::RowVectorXd RowVectorNd;
  typedef Eigen::MatrixXd MatrixNd;
  typedef Eigen::VectorXd::Index index_type;

protected:

  RowVectorNd initial_theta;
  RowVectorNd theta;
  RowVectorNd initial_std;
  RowVectorNd policy;
  RowVectorNd best_policy;
  RowVectorNd theta_eps_cur;
  RowVectorNd policy_eps;

  int n_parameter;
  int n_rolloutsEvaluated;

  boost::mt19937 rng_;
  boost::normal_distribution<> normal_dist_;
  boost::shared_ptr<boost::variate_generator<boost::mt19937, boost::normal_distribution<> > > gaussian_;

  double lambda;
  double lambdaMD;
  double expansion_factor_sigma;
  double imp_factor;
  double determinant;
  double best_cost;
  double range;

  MatrixNd covar;
  MatrixNd covar_inv;
  MatrixNd c_normalized;
  MatrixNd policy_history;
  MatrixNd theta_history;
  MatrixNd Near_policies;
  VectorNd cost_history;
  VectorNd Near_policy_costs;

  double cost2policy_cov_factor;
  double sigma;
  int nearBinsSize;
  int howManySamplesToCheck;
  bool isCostGivenBeforeNextTheta;
  bool optimizationDone;

  //! CMA-ES parameters
  double cc,ccov,chiN;
  VectorNd pc;

  //! number of policy parameters
  int memoryAllocationLength;

  //! number of initial roll-outs
  int nInitialRollouts_;


public:

  /*! Constructor
   * @param initialParameterSet : initial policy parameter vector (either a column or a row vector) from where we start searching
   * @param std : N-dimensional vector (either a column or a row vector) of initial standard deviation on each axes
   * @param initial_exp : How many samples to accumulate before the first update (minimum = 2)
   */
template <typename Derived>
Rockstar(const Eigen::MatrixBase<Derived>& initialParameterSet, const Eigen::MatrixBase<Derived>& std, int initial_exp):
isCostGivenBeforeNextTheta(true),
optimizationDone(false),
memoryAllocationLength(500000)
{
    /////////////////////////input checking/////////////////////////
    ///////////////////Probably not so interesting//////////////////
    ////////////////////////////////////////////////////////////////

    if (initialParameterSet.rows() == 1)
        initial_theta = initialParameterSet;
    else if (initialParameterSet.cols() == 1)
        initial_theta = initialParameterSet.transpose();
    else
      throw std::runtime_error("Rockstar constructor: initial parameter set should either column vector or a row vector!");

    n_parameter = initial_theta.cols();

    if (std.rows() == 1)
      initial_std = std;
    else if (std.cols() == 1)
      initial_std = std.transpose();
    else
      throw std::runtime_error("Rockstar constructor: initial standard deviation should either column vector or a row vector!");

    if (initial_theta.cols() != initial_std.cols())
      throw std::runtime_error("Rockstar constructor: the length of initial standard deviation and the length of the initial policy parameter vector should be the same!");

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    // Strategic parameters
    lambda                 = 0.2;
    lambdaMD               = 10.0;
    expansion_factor_sigma = pow(1.2,(1/log(n_parameter+2.5)))-1.0;
    imp_factor             = 1.3;
    nInitialRollouts_      = initial_exp;
    howManySamplesToCheck  = 5;

    covar                  = MatrixNd::Identity(n_parameter, n_parameter);
    covar_inv              = MatrixNd::Identity(n_parameter, n_parameter);
    c_normalized           = covar;
    cost2policy_cov_factor = gsl_cdf_chisq_Pinv(0.95,n_parameter)*-0.5/log(lambda);
    sigma                  = 1.0;
    nearBinsSize           = 0;

    // CMA parameters
    determinant            = 1.0;
    cc                     = 3.0/(n_parameter+6)/log(n_parameter+6.0);
    ccov                   = 6.0/(n_parameter+7)/log(n_parameter+7.0);
    pc                     = VectorNd::Zero(n_parameter,1);
    chiN                   = pow(n_parameter,0.5)*(1.0-1.0/(4.0*n_parameter)+1.0/pow(21.0*n_parameter,2.0));

    // initialization  !!!policy: normalized theta, theta: unnormalized original theta. covar is normalized!!!
    policy_history         = MatrixNd::Zero(memoryAllocationLength,n_parameter);
    theta_history          = MatrixNd::Zero(memoryAllocationLength,n_parameter);
    cost_history           = MatrixNd::Zero(memoryAllocationLength,1);
    policy                 = RowVectorNd::Zero(1,n_parameter);
    Near_policies          = MatrixNd::Zero( n_parameter * howManySamplesToCheck , n_parameter );
    Near_policy_costs      = VectorNd::Zero( n_parameter * howManySamplesToCheck , 1);
    theta                  = initial_theta;
    best_policy            = policy;
    best_cost              = 1.0e100;
    range                  = gsl_cdf_chisq_Pinv(0.95,policy.cols())*lambdaMD;
    n_rolloutsEvaluated    = 0;

    //initialize the random number generator
    srand(time(NULL));
    rng_.seed(rand());
    gaussian_.reset(new boost::variate_generator<boost::mt19937, boost::normal_distribution<> >(rng_, normal_dist_));

  }

  virtual ~Rockstar() {
  }

public:

  /*! getNextTheta2Evaluate
   * @param nexTheta2Evaluate : sample the next policy (either a column vector or a row vector)
   */
  template <typename Derived>
  void getNextTheta2Evaluate(Eigen::MatrixBase<Derived>& nexTheta2Evaluate){

    if(!isCostGivenBeforeNextTheta)
      throw std::runtime_error("Rockstar getNextTheta2Evaluate: Please set the cost of the previous rollout before generating a new policy!");

    // generate random vector
    RowVectorNd randn(1,n_parameter);

    for(int i=0;i<n_parameter;i++)
      randn(i)=(*gaussian_)();

    policy_eps = policy + randn * covar.llt().matrixL().transpose();

    // record the policy and theta
    policy_history.row(n_rolloutsEvaluated) = policy_eps;
    theta_eps_cur                           = policy_eps.cwiseProduct(initial_std) + initial_theta;
    theta_history.row(n_rolloutsEvaluated)  = theta_eps_cur;

    if(nexTheta2Evaluate.cols() == n_parameter)
      nexTheta2Evaluate     = theta_eps_cur;
    else if (nexTheta2Evaluate.rows() == n_parameter)
      nexTheta2Evaluate     = theta_eps_cur.transpose();
    else
      throw std::runtime_error("Rockstar getNextTheta2Evaluate: Please input a vector (either a row vector or a column vector)");

    isCostGivenBeforeNextTheta              = false;

  }

  /*! setTheCostFromTheLastTheta
   * @param cost : cost from the previous theta you got
   */
  void setTheCostFromTheLastTheta(double cost) {

    cost_history(n_rolloutsEvaluated) = cost;
    isCostGivenBeforeNextTheta        = true;

    if(cost<best_cost){
        best_cost    = cost;
        best_policy  = policy_history.row(n_rolloutsEvaluated);
    }

    if(n_rolloutsEvaluated>nInitialRollouts_-2)
      updateRegressionAndMinimum();

    n_rolloutsEvaluated++;
  }

  bool isOptimizationDone() {
    return optimizationDone;
  }

  int getRolloutNumber() {
    return n_rolloutsEvaluated;
  }

  double getBestCost(){
    return best_cost;
  }

  double getSigma(){
    return sigma;
  }

  RowVectorNd getBestEverSeenPolicy(){
    return (best_policy.cwiseProduct(initial_std) + initial_theta);
  }

  template <typename Derived>
  void getBestEverSeenPolicy(Eigen::MatrixBase<Derived>& bestPolicy){

    if(bestPolicy.cols() == n_parameter)
      bestPolicy     = best_policy.cwiseProduct(initial_std) + initial_theta;
    else if (bestPolicy.rows() == n_parameter)
      bestPolicy     = (best_policy.cwiseProduct(initial_std) + initial_theta).transpose();
    else
      throw std::runtime_error("Rockstar getBestPolicy: Please input a vector (either a row vector or a column vector)");
  }

  template <typename Derived>
  void getEstimatedOptimalPolicy(Eigen::MatrixBase<Derived>& bestPolicy){

    if(bestPolicy.cols() == n_parameter)
      bestPolicy     = policy.cwiseProduct(initial_std) + initial_theta;
    else if (bestPolicy.rows() == n_parameter)
      bestPolicy     = (policy.cwiseProduct(initial_std) + initial_theta).transpose();
    else
      throw std::runtime_error("Rockstar getEstimatedOptimalPolicy: Please input a vector (either a row vector or a column vector)");
  }

  VectorNd getCostHistory(){
    return cost_history.block(0,0,n_rolloutsEvaluated-1,1);
  }

private:

  void updateRegressionAndMinimum() {
    int counter               = 0;
    double temp_coef          = 1.0;

    //selecting policies close to the current policy
    while(true){
      for(int smaple_n = std::max(n_rolloutsEvaluated-n_parameter*howManySamplesToCheck+1,0); smaple_n <= n_rolloutsEvaluated ; smaple_n++ ){
        if(temp_coef * range > (policy_history.row(smaple_n)-policy)*covar_inv*(policy_history.row(smaple_n)-policy).transpose()){
          Near_policies.row(counter) = policy_history.row(smaple_n);
          Near_policy_costs(counter) = cost_history(smaple_n);
          counter=counter+1;
        }
      }

      if(counter > 1){
        nearBinsSize = counter;
        break;
      }

      Near_policies.setZero();
      Near_policy_costs.setZero();
      temp_coef = temp_coef * 3.0;
      counter   = 0;

    }

    RowVectorNd cur_policy_new  = RowVectorNd::Zero(1,n_parameter);
    RowVectorNd cur_policy_new2 = RowVectorNd::Zero(1,n_parameter);


    int minIndex, minIndex2;
    double E_cost  = 1e100;

    Sort(); /// sort near bins in order of increasing costs

    for(int initialN = 0; initialN < std::min(1,nearBinsSize); initialN++){
      RowVectorNd initial = Near_policies.row(initialN);
      double E_cost2 = gradientDescent(cur_policy_new2 , initial);
      if(E_cost > E_cost2){
        cur_policy_new = cur_policy_new2;
        E_cost  = E_cost2;
      }
    }

//    }

    if(cost_history(n_rolloutsEvaluated-1)>cost_history(n_rolloutsEvaluated))
        sigma = sigma * (1.0 + expansion_factor_sigma);
    else
        sigma = sigma / pow((1.0 + expansion_factor_sigma), imp_factor);

    if(sqrt((cur_policy_new-policy)*covar_inv*((cur_policy_new-policy).transpose())) < chiN*1.5){

        pc           = (1.0 - cc) * pc + cc * (cur_policy_new-policy).transpose() / sigma;
        c_normalized = (1.0 - ccov) * c_normalized + pc * pc.transpose() * ccov;
        c_normalized = c_normalized * pow(1.0 / c_normalized.determinant() , 1.0/n_parameter);
    }

    ////enforcing symmetry
    for(int k=0;k<n_parameter-1;k++)
      for(int i=k+1;i<n_parameter;i++)
        c_normalized(k,i)=c_normalized(i,k);

    covar        = c_normalized * sigma * sigma;
    covar_inv    = covar.inverse();
    policy       = cur_policy_new;

    if( sigma < 1.0e-8 )
      optimizationDone  = true;
  }

  double gradientDescent(RowVectorNd& op_policy, RowVectorNd& initial) {

    int rows      = nearBinsSize;
    int cols      = n_parameter;

    double op_cost;
    double mean_cost;


    mean_cost = Near_policy_costs.block(0,0,nearBinsSize,1).mean();

    double Pprior;
    double a;
    double b;
    double expMD2;
    double probabilistic_distance;
    MatrixNd cov_sum, alpha, residual_term, adot, bdot, Jaco, update;
    RowVectorNd diffoftheta;
    RowVectorNd policy_old;


//    Pprior        = 2.0 / pow(sqrt(2.0 * M_PI), n_parameter);
    Pprior        = 1.0;
    policy_old    = initial;
    bool terminate=false;


    //autotune alpha according to 50% in every step criteria
    alpha = 1.0 * covar * cost2policy_cov_factor / (mean_cost - Near_policy_costs.block(0,0,nearBinsSize,1).minCoeff());

    for(int iteration = 0; iteration < 200; iteration++){

      a=Pprior*mean_cost;
      b=Pprior;
      adot.setZero(1,cols);
      bdot.setZero(1,cols);

      for(int j=0;j<rows;j++){

        diffoftheta      = Near_policies.row(j) - policy_old;
        residual_term    = 0.5 * diffoftheta * covar_inv / cost2policy_cov_factor;
        expMD2           = exp(-0.25*(diffoftheta * covar_inv /cost2policy_cov_factor * diffoftheta.transpose()).sum());
        adot             = adot + ((double)Near_policy_costs(j)) * expMD2 * residual_term;
        bdot             = bdot + expMD2 * residual_term;
        a               += ((double)Near_policy_costs(j)) * expMD2;
        b               += expMD2;

      }

      if(terminate) break;

      Jaco                    = adot / b - bdot * a / b / b;
      update                 = Jaco * alpha;
      op_policy              = policy_old - update;
      probabilistic_distance  = (update*covar_inv  *update.transpose()).sum()/ cost2policy_cov_factor;
      policy_old              = op_policy;

      if(probabilistic_distance < 0.0001 || (op_policy-initial)*covar_inv / cost2policy_cov_factor *(op_policy-initial).transpose()>1.0)
        terminate = true;

    }

    op_cost   = a/b;
    return op_cost;
    }

  void Sort()
      {
        int i, j, flag = 1;    // set flag to 1 to start first pass
        int numLength = nearBinsSize;
        RowVectorNd tempV(n_parameter);
        double temp;

        for(i = 1; (i <= numLength) && flag; i++)
        {
          flag = 0;
          for (j=0; j < (numLength -1); j++)
          {
            if (Near_policy_costs(j+1) < Near_policy_costs(j))      // ascending order simply changes to <
            {
              temp = Near_policy_costs(j);             // swap elements
              Near_policy_costs(j) = Near_policy_costs(j+1);
              Near_policy_costs(j+1) = temp;
              flag = 1;               // indicates that a swap occurred.

              tempV = Near_policies.row(j);
              Near_policies.row(j) = Near_policies.row(j+1);
              Near_policies.row(j+1) = tempV;
            }
          }
        }
      }

};


} // end namespace

#endif
