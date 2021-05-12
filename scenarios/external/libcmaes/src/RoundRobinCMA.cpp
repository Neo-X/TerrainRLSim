/*
 * RoundRobinCMA.cpp
 *
 *  Created on: 2016-01-10
 *      Author: gberseth
 */

#include "RoundRobinCMA.h"
#include "cmastrategy.h"
// #include "src/esostrategy.h"

// using namespace libcmaes;

namespace libcmaes {
	// template <class TGenoPheno = CMASolutions>

	template <class TGenoPheno> using eostrat = ESOStrategy<CMAParameters<TGenoPheno>, CMASolutions, CMAStopCriteria<TGenoPheno> >;
	
	template <class TGenoPheno>
	RoundRobinCMA<TGenoPheno>::RoundRobinCMA(size_t num_members, FitFunc &func,
		CMAParameters<TGenoPheno> & parameters)
		:CMAStrategy<ACovarianceUpdate, TGenoPheno>(func, parameters)
	{
		// TODO Auto-generated constructor stub
		this->_num_members = num_members;
		this->_time = 0;
		
		for (size_t i = 0; i < this->_num_members; i++)
		{
			dVec x0 = parameters.get_x0min();
			std::vector<double> x0_; // (dim, 7.0);
			std::vector<double> x0l; // (dim, 7.0);
			std::vector<double> x0u; // (dim, 7.0);
			TGenoPheno gp_ = this->get_parameters().get_gp();
				
			for (size_t i = 0; i < parameters.dim(); i++)
			{
				double x0__ = x0(i);
				x0_.push_back(x0__);
				auto bS = gp_.get_boundstrategy();
				double x0l_ = gp_.get_boundstrategy().getPhenoLBound(i);
				double x0u_ = gp_.get_boundstrategy().getPhenoUBound(i);
				x0l.push_back(x0l_);
				x0u.push_back(x0u_);
			}
			TGenoPheno gp(&x0l.front(), &x0u.front(), parameters.dim()); // maybe this uses sigma somehow
			CMAParameters<TGenoPheno> params_(parameters.dim(), &x0_.front(), parameters.get_sigma_init(), parameters.lambda(), parameters.get_seed(), gp);
			params_.set_ftolerance(parameters.get_ftolerance());
			params_.set_xtolerance(parameters.get_xtolerance());
			params_.set_max_iter(parameters.get_max_iter());
			// <CMAStrategy<ACovarianceUpdate,TGenoPheno>,CMAParameters<TGenoPheno>,CMASolutions>
			// ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<TGenoPheno> > * optim = new ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<TGenoPheno> >(func, parameters);
			CMAStrategy<ACovarianceUpdate, TGenoPheno> * optim = new CMAStrategy<ACovarianceUpdate, TGenoPheno>(func, params_);
			ProgressFunc<CMAParameters<TGenoPheno>, CMASolutions> &pfunc = CMAStrategy<CovarianceUpdate, TGenoPheno>::_defaultPFunc;
			optim->set_progress_func(pfunc);
			// std::cout << "optim " << optim << std::endl;
			optim->eostrat<TGenoPheno>::_solutions._initial_candidate = Candidate(optim->eostrat<TGenoPheno>::_func(optim->eostrat<TGenoPheno>::_parameters._gp.pheno(optim->eostrat<TGenoPheno>::_solutions._xmean).data(), optim->eostrat<TGenoPheno>::_parameters._dim),
				optim->eostrat<TGenoPheno>::_solutions._xmean);
			optim->eostrat<TGenoPheno>::_solutions._best_seen_candidate = optim->eostrat<TGenoPheno>::_solutions._initial_candidate;
			optim->update_fevals(1);
			// const CMASolutions &solutions = CMASolutions();
			// ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> > * optim = new ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> >(func, parameters);
			this->_members.push_back(optim);

		}
		_current_member = 0;
		_round = 0;
		_degree_weight = 1.0;
		_kdmin = 20.0;
		_diversity_weight = 0.1;
		// _minimize_or_maximize = -1.0;
	}
	
	// typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy > TGenoPheno ;
	// template <class TGenoPheno = libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy >>
	/*
	// typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy > TGenoPheno;
	RoundRobinCMA::RoundRobinCMA(size_t num_members, FitFunc &func,
		double * lbounds, double * ubounds, int dim, std::vector<double> x0, size_t max_evals)
	{
		double sigma = 3.4;
		// libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
		// libcmaes::CMAParameters< libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>> cmaparams(dim, &x0.front(), sigma, -1, 0, gp);
		libcmaes::GenoPheno<pwqBoundStrategy, linScalingStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
		libcmaes::CMAParameters< GenoPheno<pwqBoundStrategy, linScalingStrategy>> cmaparams(dim, &x0.front(), sigma, -1, 0, gp);
		cmaparams.set_algo(aCMAES);
		cmaparams.set_max_fevals(max_evals);
		cmaparams.set_maximize(true);
		cmaparams.set_seed(21);

		// TODO Auto-generated constructor stub
		this->_num_members = num_members;

		for (size_t i = 0; i < this->_num_members; i++)
		{
			// <CMAStrategy<ACovarianceUpdate,TGenoPheno>,CMAParameters<TGenoPheno>,CMASolutions>
			// ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<TGenoPheno> > * optim = new ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<TGenoPheno> >(func, parameters);
			// ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>> > optim (func, cmaparams);
			// const CMASolutions &solutions = CMASolutions();
			// ESOptimizer<CMAStrategy<CovarianceUpdate>, CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>> > * optim2;
			// optim2;
			ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> > * optim = new ESOStrategy<CMAParameters<GenoPheno<pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<GenoPheno<pwqBoundStrategy, linScalingStrategy>> >(func, cmaparams);
			this->_members.push_back(optim);

		}
		_current_member = 0;
		_round = 0;
	}
	*/

	template <class TGenoPheno>
	RoundRobinCMA<TGenoPheno>::~RoundRobinCMA() {
		// TODO Auto-generated destructor stub

		for (size_t i = 0; i < this->_num_members; i++)
		{
			// delete this->_members.at(i); // Protected??
		}
	}

	template <class TGenoPheno>
	dMat RoundRobinCMA<TGenoPheno>::ask()
	{
		// return this->_members[_current_member]->ask();
		CMAStrategy<ACovarianceUpdate, TGenoPheno> * mem = this->_members[_current_member];
		// compute eigenvalues and eigenvectors.
		if (!mem->eostrat<TGenoPheno>::_parameters._sep && !mem->eostrat<TGenoPheno>::_parameters._vd)
		{
			mem->eostrat<TGenoPheno>::_solutions._updated_eigen = false;
			if (mem->eostrat<TGenoPheno>::_niter == 0 || !mem->eostrat<TGenoPheno>::_parameters._lazy_update
				|| mem->eostrat<TGenoPheno>::_niter - mem->eostrat<TGenoPheno>::_solutions._eigeniter > mem->eostrat<TGenoPheno>::_parameters._lazy_value)
			{
				mem->eostrat<TGenoPheno>::_solutions._eigeniter = mem->eostrat<TGenoPheno>::_niter;
				mem->_esolver.setMean(mem->eostrat<TGenoPheno>::_solutions._xmean);
				mem->_esolver.setCovar(mem->eostrat<TGenoPheno>::_solutions._cov);
				mem->eostrat<TGenoPheno>::_solutions._updated_eigen = true;
			}
		}
		else if (mem->eostrat<TGenoPheno>::_parameters._sep)
		{
			mem->_esolver.setMean(mem->eostrat<TGenoPheno>::_solutions._xmean);
			mem->_esolver.set_covar(mem->eostrat<TGenoPheno>::_solutions._sepcov);
			mem->_esolver.set_transform(mem->eostrat<TGenoPheno>::_solutions._sepcov.cwiseSqrt());
		}
		else if (mem->eostrat<TGenoPheno>::_parameters._vd)
		{
			mem->_esolver.setMean(mem->eostrat<TGenoPheno>::_solutions._xmean);
			mem->_esolver.set_covar(mem->eostrat<TGenoPheno>::_solutions._sepcov);
		}

		//debug
		//std::cout << "transform: " << _esolver._transform << std::endl;
		//debug

		// sample for multivariate normal distribution, produces one candidate per column.
		dMat pop;
		if (!mem->eostrat<TGenoPheno>::_parameters._sep && !mem->eostrat<TGenoPheno>::_parameters._vd)
			pop = mem->_esolver.samples(mem->eostrat<TGenoPheno>::_parameters._lambda, mem->eostrat<TGenoPheno>::_solutions._sigma); // Eq (1).
		else if (mem->eostrat<TGenoPheno>::_parameters._sep)
			pop = mem->_esolver.samples_ind(mem->eostrat<TGenoPheno>::_parameters._lambda, mem->eostrat<TGenoPheno>::_solutions._sigma);
		else if (mem->eostrat<TGenoPheno>::_parameters._vd)
		{
			pop = mem->_esolver.samples_ind(mem->eostrat<TGenoPheno>::_parameters._lambda);
			double normv = mem->eostrat<TGenoPheno>::_solutions._v.squaredNorm();
			double fact = std::sqrt(1 + normv) - 1;
			dVec vbar = mem->eostrat<TGenoPheno>::_solutions._v / std::sqrt(normv);

			pop += fact * vbar * (vbar.transpose() * pop);
			for (int i = 0;i<pop.cols();i++)
			{
				pop.col(i) = mem->eostrat<TGenoPheno>::_solutions._xmean + mem->eostrat<TGenoPheno>::_solutions._sigma * mem->eostrat<TGenoPheno>::_solutions._sepcov.cwiseProduct(pop.col(i));
			}
		}

		// gradient if available.
		if (mem->eostrat<TGenoPheno>::_parameters._with_gradient)
		{
			dVec grad_at_mean = mem->eostrat<TGenoPheno>::gradf(mem->eostrat<TGenoPheno>::_parameters._gp.pheno(mem->eostrat<TGenoPheno>::_solutions._xmean));
			dVec gradgp_at_mean = mem->eostrat<TGenoPheno>::gradgp(mem->eostrat<TGenoPheno>::_solutions._xmean); // for geno / pheno transform.
			grad_at_mean = grad_at_mean.cwiseProduct(gradgp_at_mean);
			if (grad_at_mean != dVec::Zero(mem->eostrat<TGenoPheno>::_parameters._dim))
			{
				dVec nx;
				if (!mem->eostrat<TGenoPheno>::_parameters._sep && !mem->eostrat<TGenoPheno>::_parameters._vd)
				{
					dMat sqrtcov = mem->_esolver._eigenSolver.operatorSqrt();
					dVec q = sqrtcov * grad_at_mean;
					double normq = q.squaredNorm();
					nx = mem->eostrat<TGenoPheno>::_solutions._xmean - mem->eostrat<TGenoPheno>::_solutions._sigma * (sqrt(mem->eostrat<TGenoPheno>::_parameters._dim / normq)) * mem->eostrat<TGenoPheno>::_solutions._cov * grad_at_mean;
				}
				else nx = mem->eostrat<TGenoPheno>::_solutions._xmean - mem->eostrat<TGenoPheno>::_solutions._sigma * (sqrt(mem->eostrat<TGenoPheno>::_parameters._dim) / ((mem->eostrat<TGenoPheno>::_solutions._sepcov.cwiseSqrt().cwiseProduct(grad_at_mean)).norm())) * mem->eostrat<TGenoPheno>::_solutions._sepcov.cwiseProduct(grad_at_mean);
				pop.col(0) = nx;
			}
		}

		// tpa: fill up two first (or second in case of gradient) points with candidates usable for tpa computation
		if (mem->eostrat<TGenoPheno>::_parameters._tpa == 2 && mem->eostrat<TGenoPheno>::_niter > 0)
		{
			dVec mean_shift = mem->eostrat<TGenoPheno>::_solutions._xmean - mem->eostrat<TGenoPheno>::_solutions._xmean_prev;
			double mean_shift_norm = 1.0;
			if (!mem->eostrat<TGenoPheno>::_parameters._sep && !mem->eostrat<TGenoPheno>::_parameters._vd)
				mean_shift_norm = (mem->_esolver._eigenSolver.eigenvalues().cwiseSqrt().cwiseInverse().cwiseProduct(mem->_esolver._eigenSolver.eigenvectors().transpose()*mean_shift)).norm() / mem->eostrat<TGenoPheno>::_solutions._sigma;
			else mean_shift_norm = mem->eostrat<TGenoPheno>::_solutions._sepcov.cwiseSqrt().cwiseInverse().cwiseProduct(mean_shift).norm() / mem->eostrat<TGenoPheno>::_solutions._sigma;
			//std::cout << "mean_shift_norm=" << mean_shift_norm << " / sqrt(N)=" << std::sqrt(std::sqrt(mem->eostrat<TGenoPheno>::_parameters._dim)) << std::endl;

			dMat rz = mem->_esolver.samples_ind(1);
			double mfactor = rz.norm();
			dVec z = mfactor * (mean_shift / mean_shift_norm);
			mem->eostrat<TGenoPheno>::_solutions._tpa_x1 = mem->eostrat<TGenoPheno>::_solutions._xmean + z;
			mem->eostrat<TGenoPheno>::_solutions._tpa_x2 = mem->eostrat<TGenoPheno>::_solutions._xmean - z;

			// if gradient is in col 0, move tpa vectors to pos 1 & 2
			if (mem->eostrat<TGenoPheno>::_parameters._with_gradient)
			{
				mem->eostrat<TGenoPheno>::_solutions._tpa_p1 = 1;
				mem->eostrat<TGenoPheno>::_solutions._tpa_p2 = 2;
			}
			pop.col(mem->eostrat<TGenoPheno>::_solutions._tpa_p1) = mem->eostrat<TGenoPheno>::_solutions._tpa_x1;
			pop.col(mem->eostrat<TGenoPheno>::_solutions._tpa_p2) = mem->eostrat<TGenoPheno>::_solutions._tpa_x2;
		}

		// if some parameters are fixed, reset them.
		if (!mem->eostrat<TGenoPheno>::_parameters._fixed_p.empty())
		{
			for (auto it = mem->eostrat<TGenoPheno>::_parameters._fixed_p.begin();
			it != mem->eostrat<TGenoPheno>::_parameters._fixed_p.end();++it)
			{
				pop.block((*it).first, 0, 1, pop.cols()) = dVec::Constant(pop.cols(), (*it).second).transpose();
			}
		}

		return pop;
	}
	
	template <class TGenoPheno>
	void RoundRobinCMA<TGenoPheno>::eval(const dMat &candidates, const dMat &phenocandidates)
	{
		// custom eval.
		auto optim = this->_members[_current_member];
		std::chrono::time_point<std::chrono::system_clock> tstart = std::chrono::system_clock::now();
		const dMat &phenocandidates_ = optim->eostrat<TGenoPheno>::_parameters._gp.pheno(candidates);
		const dMat &candidates_ = candidates;
		double maxDiversity = -10000000;
		double minDiversityMin = 10000000;
		double maxMetric = -10000000;
		for (int r = 0;r<candidates_.cols();r++)
		{

			optim->_solutions._candidates.at(r).set_x(candidates_.col(r));
			optim->_solutions._candidates.at(r).set_id(r);
			double f_value;
			const double *x;
			if (phenocandidates_.size())
			{
				x = phenocandidates_.col(r).data();
			}
			else
			{
				x = candidates_.col(r).data();
			}
			f_value = optim->_func(x, candidates_.rows());
			// f_value = f_value*this->_degree_weight;
			if (f_value > maxMetric)
			{
				maxMetric = f_value;
			}

			double diversity = -10000.0; // upper bound best guess
			double diversityMin = 0.0; // upper bound best guess
			if (_round > 0)
			{
				diversity = diversityMetric(x, candidates_.rows(), _current_member);
				diversity = diversity*this->_diversity_weight;
				diversityMin = diversityMinMetric(x, candidates_.rows(), _current_member, 0.5);
				diversityMin = -diversityMin*this->_kdmin;
				// std::cout << "f_value: " << f_value << " Diversity: " << diversity << " DiversityMin: " << diversityMin << std::endl;
			}
			double total_f = (f_value) + (diversity) + (diversityMin); // for maximization
																			   // double total_f = f_value + (-diversityMin*20.0);
			optim->get_solutions().get_candidate(r).set_fvalue(-total_f);
			if ((diversity) > maxDiversity)
			{
				maxDiversity = diversity;
			}
			if ((diversityMin) < minDiversityMin)
			{
				minDiversityMin = diversityMin;
			}

			// optim->_solutions._candidates.at(r).set_fvalue(f);
			//std::cerr << "candidate x: " << _solutions._candidates.at(r)._x.transpose() << std::endl;
		}
		int nfcalls = candidates.cols();

		/*
		// evaluation step of uncertainty handling scheme.
		if (optim->_parameters._uh && false) // Not supported 
		{
			// compute the number of solutions to re-evaluate
			optim->_solutions._lambda_reev = 0.0;
			double r_l = optim->_parameters._rlambda * optim->_parameters._lambda;
			int lr_l = std::floor(r_l);
			double pr_l = r_l - lr_l;
			double p = optim->_uhunif(optim->_uhgen);
			if (p < pr_l)
				optim->_solutions._lambda_reev = lr_l + 1;
			else optim->_solutions._lambda_reev = lr_l;
			if (optim->_solutions._lambda_reev == 0)
				optim->_solutions._lambda_reev = 1;

			// mutate candidates.
			dMat ncandidates;
			if (phenocandidates.size())
				ncandidates = phenocandidates.block(0, 0, phenocandidates.rows(), optim->_solutions._lambda_reev);
			else ncandidates = candidates.block(0, 0, candidates.rows(), optim->_solutions._lambda_reev);
			if (optim->_solutions._sepcov.size())
				optim->_uhesolver.set_covar(optim->_solutions._sepcov);
			else optim->_uhesolver.set_covar(optim->_solutions._cov);
			ncandidates += optim->_parameters._epsuh * optim->_solutions._sigma * optim->_uhesolver.samples_ind(optim->_solutions._lambda_reev);

			// re-evaluate
			std::vector<RankedCandidate> nvcandidates;
			for (int r = 0;r<candidates.cols();r++)
			{
				if (r < optim->_solutions._lambda_reev)
				{
					double nfvalue = optim->_func(ncandidates.col(r).data(), ncandidates.rows());
					nvcandidates.emplace_back(nfvalue, optim->_solutions._candidates.at(r), r);
					nfcalls++;
				}
				else nvcandidates.emplace_back(optim->_solutions._candidates.at(r).get_fvalue(), optim->_solutions._candidates.at(r), r);
			}
			optim->_solutions._candidates_uh = nvcandidates;
		}

		// if an elitist is active, reinject initial solution as needed.
		if (optim->_niter > 0 && (optim->_parameters._elitist || optim->_parameters._initial_elitist || (optim->_initial_elitist && optim->_parameters._initial_elitist_on_restart)))
		{
			// get reference values.
			double ref_fvalue = std::numeric_limits<double>::max();
			Candidate ref_candidate;

			if (optim->_parameters._initial_elitist_on_restart || optim->_parameters._initial_elitist)
			{
				ref_fvalue = optim->_solutions._initial_candidate.get_fvalue();
				ref_candidate = optim->_solutions._initial_candidate;
			}
			else if (optim->_parameters._elitist)
			{
				ref_fvalue = optim->_solutions._best_seen_candidate.get_fvalue();
				ref_candidate = optim->_solutions._best_seen_candidate;
			}

			// reinject intial solution if half or more points have value above that of the initial point candidate.
			int count = 0;
			for (int r = 0;r<candidates.cols();r++)
				if (optim->_solutions._candidates.at(r).get_fvalue() < ref_fvalue)
					++count;
			if (count / 2.0 < candidates.cols() / 2)
			{
				optim->_solutions._candidates.at(1) = ref_candidate;
			}
		}
		*/
		std::chrono::time_point<std::chrono::system_clock> tstop = std::chrono::system_clock::now();
		eostrat<TGenoPheno>::_solutions._elapsed_last_iter = std::chrono::duration_cast<std::chrono::milliseconds>(tstop - tstart).count();
		optim->eostrat<TGenoPheno>::_solutions._elapsed_last_iter = std::chrono::duration_cast<std::chrono::milliseconds>(tstop - tstart).count();
		this->_time += std::chrono::duration_cast<std::chrono::milliseconds>(tstop - tstart).count();


		optim->update_fevals(nfcalls);

		// std::cout << "Best f value: " <<  optim->get_solutions().get_best_seen_candidate().get_fvalue() << std::endl;
		// optim->tell();

		optim->inc_iter();
		this->_diversityMin = minDiversityMin;
		this->_diversity = maxDiversity;
		this->_metric = maxMetric;

	}
	
	template <class TGenoPheno>
	double RoundRobinCMA<TGenoPheno>::diversityMetric(const double *x, const int N, const size_t member)
	{
		double sum = 0;
		TGenoPheno gp_ = this->get_parameters().get_gp();

		for (size_t i = 0; i < this->_num_members; i++)
		{
			if (i == member)
			{
				continue;
			}
			CMASolutions& best_s = this->_members[i]->get_solutions();

			dVec x_m = this->get_parameters().get_gp().pheno(best_s.best_candidate().get_x_dvec());
			double tmp_sum = 0;
			for (int n = 0; n < N; n++)
			{
				double x0l_ = gp_.get_boundstrategy().getPhenoLBound(n);
				double x0u_ = gp_.get_boundstrategy().getPhenoUBound(n);
				// normalize values between 0 and 1;
				double x_ = (x[n] - x0l_) / (x0u_ - x0l_);
				double x_m_ = (x_m[n] - x0l_) / (x0u_ - x0l_);
				double d = (x_ - x_m_);
				// std::cout << "d: " << d << std::endl;
				tmp_sum += d*d;
			}
			// std::cout << "tmp_sum"
			sum += sqrt(tmp_sum);
		}
		return sum;

	}

	template <class TGenoPheno>
	double RoundRobinCMA<TGenoPheno>::diversityMinMetric(const double *x, const int N, const size_t member, double mind)
	{
		double min = 0;
		TGenoPheno gp_ = this->get_parameters().get_gp();
		bool set_min = false;
		for (size_t i = 0; i < this->_num_members; i++)
		{
			if (i == member)
			{
				continue;
			}
			CMASolutions& best_s = this->_members[i]->get_solutions();
			dVec x_m = this->get_parameters().get_gp().pheno(best_s.best_candidate().get_x_dvec());
			double sum_tmp = 0;
			for (int n = 0; n < N; n++)
			{
				double x0l_ = gp_.get_boundstrategy().getPhenoLBound(n);
				double x0u_ = gp_.get_boundstrategy().getPhenoUBound(n);
				// normalize values between 0 and 1;
				double x_ = (x[n] - x0l_) / (x0u_ - x0l_);
				double x_m_ = (x_m[n] - x0l_) / (x0u_ - x0l_);
				// std::cout << "x[n]: " << x[n] << " x_m[n]: " << x_m[n] << " x0u_: " << x0u_ << " x0l_: " << x0l_ << "x_: " << x_ << " x_m_: " << x_m_ << std::endl;
				double d = (x_ - x_m_);
				sum_tmp += d*d;
			}
			sum_tmp = sqrt(sum_tmp);
			if (sum_tmp < mind)
			{
				double d_ = mind - sum_tmp;
				if (d_ > min)
				{
					min = d_;
				}
				set_min = true;
			}

		}
		if (!set_min)
		{
			return 0;
		}
		return min;
	}
	
	template <class TGenoPheno>
	void RoundRobinCMA<TGenoPheno>::tell()
	{
		// this->_members[_current_member]->tell();
		//debug
		//DLOG(INFO) << "tell()\n";
		//debug
		CMAStrategy<ACovarianceUpdate, TGenoPheno> * mem = this->_members[_current_member];

#ifdef HAVE_DEBUG
		std::chrono::time_point<std::chrono::system_clock> tstart = std::chrono::system_clock::now();
#endif

		// sort candidates.
		if (!mem->eostrat<TGenoPheno>::_parameters._uh)
			mem->eostrat<TGenoPheno>::_solutions.sort_candidates();
		else mem->eostrat<TGenoPheno>::uncertainty_handling();

		// call on tpa computation of s(t)
		if (mem->eostrat<TGenoPheno>::_parameters._tpa == 2 && mem->eostrat<TGenoPheno>::_niter > 0)
			mem->eostrat<TGenoPheno>::tpa_update();

		// update function value history, as needed.
		mem->eostrat<TGenoPheno>::_solutions.update_best_candidates();

		// CMA-ES update, depends on the selected 'flavor'.
		ACovarianceUpdate::update(mem->eostrat<TGenoPheno>::_parameters, mem->_esolver, mem->eostrat<TGenoPheno>::_solutions);

		if (mem->eostrat<TGenoPheno>::_parameters._uh)
			if (mem->eostrat<TGenoPheno>::_solutions._suh > 0.0)
				mem->eostrat<TGenoPheno>::_solutions._sigma *= mem->eostrat<TGenoPheno>::_parameters._alphathuh;

		// other stuff.
		if (!mem->eostrat<TGenoPheno>::_parameters._sep && !mem->eostrat<TGenoPheno>::_parameters._vd)
			mem->eostrat<TGenoPheno>::_solutions.update_eigenv(mem->_esolver._eigenSolver.eigenvalues(),
				mem->_esolver._eigenSolver.eigenvectors());
		else mem->eostrat<TGenoPheno>::_solutions.update_eigenv(mem->eostrat<TGenoPheno>::_solutions._sepcov,
			dMat::Constant(mem->eostrat<TGenoPheno>::_parameters._dim, 1, 1.0));
#ifdef HAVE_DEBUG
		std::chrono::time_point<std::chrono::system_clock> tstop = std::chrono::system_clock::now();
		eostrat<TGenoPheno>::_solutions._elapsed_tell = std::chrono::duration_cast<std::chrono::milliseconds>(tstop - tstart).count();
#endif

		_current_member++;
		_current_member = _current_member % _num_members;
		if (0 == (_current_member % _num_members))
		{
			_round++;
		}
	}
	
	template <class TGenoPheno>
	bool RoundRobinCMA<TGenoPheno>::stop()
	{
		bool stop_ = true;
		for (size_t member = 0; member < this->_num_members; member++)
		{
			stop_ = stop_ && this->_stop(member);
		}
		return stop_;
	}


	template <class TGenoPheno>
	bool RoundRobinCMA<TGenoPheno>::_stop(size_t member)
	{
		CMAStrategy<ACovarianceUpdate, TGenoPheno> * mem = this->_members[0];
		/*
		bool stop = true;
		for (size_t i = 0; i < this->_members.size(); i++)
		{
			stop = stop && this->_members[i]->stop();
		}
		return stop;
		*/
		return this->_members[member]->stop();
		/*
		if (mem->eostrat<TGenoPheno>::_solutions._run_status < 0) // an error occured, most likely out of memory at cov matrix creation.
			return true;

		if (mem->eostrat<TGenoPheno>::_pfunc(mem->eostrat<TGenoPheno>::_parameters, mem->eostrat<TGenoPheno>::_solutions)) // progress function.
			return true; // end on progress function internal termination, possibly custom.
			*/

// if (!mem->eostrat<TGenoPheno>::_parameters._fplot.empty())
	//		plot();
/*
		if (mem->eostrat<TGenoPheno>::_niter == 0)
			return false;

		if ((mem->eostrat<TGenoPheno>::_solutions._run_status =  mem->_stopcriteria.stop(mem->eostrat<TGenoPheno>::_parameters, mem->eostrat<TGenoPheno>::_solutions)) != CONT)
			return true;
			*/
		// else return false;
	}
	

	template <class TGenoPheno>
	void RoundRobinCMA<TGenoPheno>::stopReason()
	{
		for (size_t i = 0; i < this->_num_members; i++)
		{
			CMASolutions best_s = this->_members[i]->get_solutions();
			std::cout << best_s.run_status() << std::endl;
			// std::cout << "termination reason: " << CMAStopCriteria<TGenoPheno>::_scriterias.find(best_s.run_status())->second << std::endl;
		}
	}

	template <class TGenoPheno>
	std::vector<std::vector<double> > RoundRobinCMA<TGenoPheno>::get_solutions()
	{
		TGenoPheno gp_ = this->get_parameters().get_gp();
		// double x0l_ = gp_.get_boundstrategy().getPhenoLBound(i);
		std::vector<std::vector<double> > out;
		for (size_t i = 0; i < this->_num_members; i++)
		{
			CMASolutions best_s = this->getSolution(i);
			dVec x_m = this->get_parameters().get_gp().pheno(best_s.best_candidate().get_x_dvec());
			std::vector<double> params;
			params.assign(x_m.data(),x_m.data()+x_m.size());
			// std::vector<double> params = gp_.pheno(best_s.best_candidate().get_x_dvec()).get_x();
			out.push_back(params);
		}
		return out;
	}

	template <class TGenoPheno>
	CMASolutions RoundRobinCMA<TGenoPheno>::getSolution(size_t m)
	{
		return this->_members[m]->get_solutions();
	}

	template <class TGenoPheno>
	double RoundRobinCMA<TGenoPheno>::getMemberDiversity(size_t m)
	{
		dVec x = this->get_parameters().get_gp().pheno(this->getSolution(m).best_candidate().get_x_dvec());
		const double * x_ = x.data();
		return diversityMetric(x_, this->getSolution(m).best_candidate().get_x_size(), m);
	}

	template class RoundRobinCMA<GenoPheno<pwqBoundStrategy, linScalingStrategy>>;
	template class RoundRobinCMA<GenoPheno<NoBoundStrategy, NoScalingStrategy>>;
	template class RoundRobinCMA<GenoPheno<pwqBoundStrategy, NoScalingStrategy>>;
	template class RoundRobinCMA<GenoPheno<NoBoundStrategy, linScalingStrategy>>;
	// template class ESOStrategy<CMAParameters<TGenoPheno>, CMASolutions, CMAStopCriteria<TGenoPheno> >;
	// template class ESOStrategy<CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>> >;
	// template class ESOStrategy<CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>>, CMASolutions, CMAStopCriteria<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>> >;
	// template class libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>;
}
