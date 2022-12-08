

#pragma once


#include <nv/mat.h>
#include <string>
#include <vector>

#include <ceres/problem.h>

#include <nv/refinement/cost.h>
#include <nv/timer.h>

namespace nv
{

    /**
     * @brief   Nonlinear Least Squares solver class, which
     *          which encapsulates Ceres Solver for easier and
     *          more flexible usage.
     * @author  Robert Maier <robert.maier@tum.de>
     */
	class NLSSolver
	{
	public:

		struct ProblemInfo
		{
			ProblemInfo();

            size_t iteration;
			size_t residuals;
			size_t parameters;
			double cost;
            size_t residual_types;
            std::vector<size_t> type_residuals;
            std::vector<double> type_costs;
            std::vector<double> type_weights;
            double time_add;
            double time_build;

            std::string toString(bool print_costs = true) const;
		};


		struct SolverInfo
		{
			SolverInfo();

            size_t iteration;
			double cost;
            double cost_final;
            double cost_change;
            size_t inner_iterations;
            double trust_region_radius;
			std::string report;
            double time_solve;

			std::string toString() const;
		};


		NLSSolver();
		~NLSSolver();

        bool reset(size_t num_cost_types = 1);

		bool addResidual(VoxelResidual &residual);
        bool addResidual(size_t cost_id, const VoxelResidual &residual);
        void setCostWeight(size_t cost_id, double weight);
        double costWeight(size_t cost_id);

        void setDebug(bool debug);

        bool buildProblem(bool use_normalized_weights = false);
        bool solve(int lm_steps);

		bool fixParamBlock(double* ptr);

	private:
		std::vector<double> normalizeCostTermWeights();

		void removeInvalidResiduals();

        size_t num_cost_types_;
		ceres::Problem* problem_;
		std::vector< std::vector<VoxelResidual> > residuals_;
        std::vector<double> cost_type_weights_;

        std::vector<ProblemInfo> problem_info_;
        std::vector<SolverInfo> solver_info_;
		Timer tmr_;
        bool debug_;
        bool calculate_type_costs_;
	};

} 
