#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>

#include <functional>
#include <random>

bool assert_SO3(const Eigen::Ref<const Eigen::Matrix<double, 3, 3> > &R);


class Rand_double
{
	public:
		Rand_double(double low, double high)
			:r(std::bind(std::uniform_real_distribution<>(low,high),std::default_random_engine())){}

		double operator()(){ return r(); }

	private:
		std::function<double()> r;
};

#endif
