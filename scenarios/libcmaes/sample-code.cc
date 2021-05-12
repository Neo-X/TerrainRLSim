/**
 * CMA-ES, Covariance Matrix Adaptation Evolution Strategy
 * Copyright (c) 2014 Inria
 * Author: Emmanuel Benazera <emmanuel.benazera@lri.fr>
 *
 * This file is part of libcmaes.
 *
 * libcmaes is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libcmaes is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libcmaes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cmaes.h"
#include "RoundRobinCMA.h"
#include <iostream>

using namespace libcmaes;

FitFunc fsphere = [](const double *x, const int N)
{
  double val = 0.0;
  for (int i=0;i<N;i++)
    val += x[i]*x[i];
  return -val;
};

FitFunc fpoint = [](const double *x, const int N)
{
  double val = 0.0;
  for (int i=0;i<N;i++)
    val += x[i]+x[i];
  return val;
};

class customCMAStrategy : public CMAStrategy<CovarianceUpdate>
{
public:
  customCMAStrategy(FitFunc &func,
		    CMAParameters<> &parameters)
    :CMAStrategy<CovarianceUpdate>(func,parameters)
  {
  }

  ~customCMAStrategy() {}

  dMat ask()
  {
    return CMAStrategy<CovarianceUpdate>::ask();
  }

  void eval(const dMat &candidates,
	    const dMat &phenocandidates=dMat(0,0))
  {
    // custom eval.
    for (int r=0;r<candidates.cols();r++)
      {
	_solutions.get_candidate(r).set_x(candidates.col(r));
	if (phenocandidates.size()) // if candidates in phenotype space are given
	  _solutions.get_candidate(r).set_fvalue(_func(phenocandidates.col(r).data(),candidates.rows()));
	else _solutions.get_candidate(r).set_fvalue(_func(candidates.col(r).data(),candidates.rows()));

	//std::cerr << "candidate x: " << _solutions.get_candidate(r).get_x_dvec().transpose() << std::endl;
      }
    update_fevals(candidates.cols());
  }

  void tell()
  {
    CMAStrategy<CovarianceUpdate>::tell();
  }

  bool stop()
  {
    return CMAStrategy<CovarianceUpdate>::stop();
  }

};

int boundExample()
{
  const int dim = 10; // problem dimensions.
  double sigma = 0.1;
  double lbounds[dim],ubounds[dim]; // arrays for lower and upper parameter bounds, respectively
  for (int i=0;i<dim;i++)
    {
      lbounds[i] = -2.0;
      ubounds[i] = 2.0;
    }
  std::vector<double> x0(dim,1.0); // beware that x0 is within bounds.
  GenoPheno<pwqBoundStrategy> gp(lbounds,ubounds,dim); // genotype / phenotype transform associated to bounds.
  CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(x0,sigma,-1,0,gp); // -1 for automatically decided lambda, 0 is for random seeding of the internal generator.
  cmaparams.set_algo(aCMAES);
  CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(fsphere,cmaparams);
  std::cout << "best solution: " << cmasols << std::endl;
  std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
  return cmasols.run_status();
}

void roundRobinExample()
{
	int dim = 2; // problem dimensions.
	  std::vector<double> x0;
	  double sigma = 0.1;
	  size_t members = 40;

	  double * lbounds = new double[dim];
	  double * ubounds = new double[dim]; // arrays for lower and upper parameter bounds, respectively
	  std::vector<double> sigma_; // (dim, 7.0);
	  int offsrping = 10; // lambda
	  for (size_t p = 0; p < dim; p++)
	  {
		  x0.push_back(0);
		  lbounds[p] = -2;
		  ubounds[p] = 3;
		  sigma_.push_back((ubounds[p] - lbounds[p])*sigma);
	  }
	  libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
	  libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> > cmaparams(dim, &x0.front(), sigma, offsrping, 21, gp);
	  cmaparams.set_algo(aCMAES);
	  // cmaparams.set_max_fevals(1000);
	  // cmaparams.set_maximize(true);
	  cmaparams.set_seed(21);// cmaparams.set_max_fevals(1000000);
	  // cmaparams.set_
	  //ESOptimizer<CMAStrategy<CovarianceUpdate>,CMAParameters<>> optim(fsphere,cmaparams);
	  RoundRobinCMA<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>> optim(members, fsphere, cmaparams);

	  while(!optim.stop())
	    {
			size_t member = optim._current_member;
			dMat candidates = optim.ask();
			const dMat &phenocandidates = dMat(0, 0);
			optim.eval(candidates, phenocandidates);
			optim.tell();
	      // optim.tell();
	      // optim.inc_iter(); // important step: signals next iteration.
	    }
	  std::vector<std::vector<double> > sols = optim.get_solutions();
	  for (size_t r=0; r < sols.size(); r++)
	  {
		  size_t c=0;

		  for (; c < sols.at(r).size()-1; c++)
		  {
			  std::cout << sols.at(r).at(c) << ", ";
		  }
		  std::cout << sols.at(r).at(c) << std::endl;
	  }

	  // optim.stopReason();
}

int main(int argc, char *argv[])
{

	// boundExample();
	roundRobinExample();
	return 0;
}
