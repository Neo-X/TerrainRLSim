/*
 * RoundRobinCMA.h
 *
 *  Created on: 2016-01-10
 *      Author: gberseth
 */

#ifndef ROUNDROBINCMA_H_
#define ROUNDROBINCMA_H_

#include "src/cmaes.h"
#include <iostream>
#include <vector>
// #include "SteerOptPlugin.h"

using namespace libcmaes;


namespace libcmaes {
	
	// template class ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> >;
	// template class ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> >;
	template <class TGenoPheno>
	class RoundRobinCMA : public CMAStrategy<ACovarianceUpdate, TGenoPheno>// public ESOStrategy<CMAParameters<TGenoPheno>, CMASolutions, CMAStopCriteria<TGenoPheno> >
	{
		
	public:
		// template <class TGenoPheno = GenoPheno<NoBoundStrategy>>;
		// template <class TGenoPheno = libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>>;
		// typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy > TGenoPheno;

		RoundRobinCMA(size_t num_members, FitFunc &func,
			CMAParameters<TGenoPheno> &parameters);
		// template <class TGenoPheno = libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy >>
		// RoundRobinCMA(size_t num_members, FitFunc &func,
			// double * lbounds, double * ubounds, int dim, std::vector<double> x0, size_t max_evals);
		
		virtual ~RoundRobinCMA();

		virtual void eval(const dMat &candidates, const dMat &phenocandidates);

		virtual dMat ask();
		virtual void tell();
		virtual bool stop();
		virtual bool _stop(size_t member);
		virtual void stopReason();
		virtual double diversityMetric(const double *x, const int N, const size_t member);
		virtual double diversityMinMetric(const double *x, const int N, const size_t member, double mind);

		// template<class TParameters,class TSolutions=CMASolutions>
		virtual std::vector<std::vector<double> > get_solutions();
		virtual CMASolutions getSolution(size_t m);
		virtual double getMemberDiversity(size_t m);

		size_t _num_members;
		size_t _current_member;
		std::vector  <CMAStrategy<ACovarianceUpdate, TGenoPheno> *> _members;
		size_t _round;
		double _diversityMin;
		double _diversity;
		double _metric;
		size_t _time;
		double _kdmin; // diversity min distance weight
		double _diversity_weight; // weight for the diversity metric
		double _degree_weight; // weight for the diversity metric
		// double _minimize_or_maximize; // -1.0 for mazimizationa dn 1.0 for minimization

	private:


	};

}

#endif /* ROUNDROBINCMA_H_ */
