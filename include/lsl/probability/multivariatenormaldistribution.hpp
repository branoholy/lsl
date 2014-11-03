/*
 * LIDAR System Library
 * Copyright (C) 2014  Branislav Holý <branoholy@gmail.com>
 *
 * This file is part of LIDAR System Library.
 *
 * LIDAR System Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LIDAR System Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LIDAR System Library.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef LSL_PROBABILITY_MULTIVARIATENORMALDISTRIBUTION_HPP
#define LSL_PROBABILITY_MULTIVARIATENORMALDISTRIBUTION_HPP

#include <initializer_list>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "lsl/geom/vector.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace probability {

template<unsigned int dim>
class MultivariateNormalDistribution
{
private:
	Eigen::Matrix<double, dim, 1> meanVector;
	Eigen::Matrix<double, dim, dim> covarianceMatrix;
	double covarianceMatrixDeterminant;
	Eigen::Matrix<double, dim, dim> inverseCovarianceMatrix;

	double two_pi_k;

	void preComputeCovarianceMatrix();

public:
	MultivariateNormalDistribution(const Eigen::Matrix<double, dim, 1>& meanVector, const Eigen::Matrix<double, dim, dim>& covarianceMatrix);
	MultivariateNormalDistribution(const std::vector<geom::Vector<double, dim>>& points);

	inline Eigen::Matrix<double, dim, 1> getMeanVector() const { return meanVector; }
	inline void setMeanVector(const Eigen::Matrix<double, dim, 1>& meanVector) { this->meanVector = meanVector; }
	inline Eigen::Matrix<double, dim, dim> getCovarianceMatrix() const { return covarianceMatrix; }
	void setCovarianceMatrix(const Eigen::Matrix<double, dim, dim>& covarianceMatrix);

	double operator()(const Eigen::Matrix<double, dim, 1>& x) const;
};

template<unsigned int dim>
MultivariateNormalDistribution<dim>::MultivariateNormalDistribution(const Eigen::Matrix<double, dim, 1>& meanVector, const Eigen::Matrix<double, dim, dim>& covarianceMatrix) :
	meanVector(meanVector), covarianceMatrix(covarianceMatrix)
{
	two_pi_k = std::pow(utils::MathUtils::TWO_PI, dim);
	preComputeCovarianceMatrix();
}

template<unsigned int dim>
MultivariateNormalDistribution<dim>::MultivariateNormalDistribution(const std::vector<geom::Vector<double, dim>>& points)
{
	two_pi_k = std::pow(utils::MathUtils::TWO_PI, dim);

	meanVector = Eigen::Matrix<double, dim, 1>::Zero();
	for(const geom::Vector<double, dim>& p : points)
	{
		for(unsigned int d = 0; d < dim; d++)
		{
			meanVector(d) += p.get(d);
		}
	}
	meanVector /= points.size();

	covarianceMatrix = Eigen::Matrix<double, dim, dim>::Zero();
	for(const geom::Vector<double, dim>& p : points)
	{
		Eigen::Matrix<double, dim, 1> diff(p.get(0), p.get(1));
		diff -= meanVector;
		covarianceMatrix += diff * diff.transpose();
	}
	covarianceMatrix /= points.size() - 1;
	preComputeCovarianceMatrix();
}

template<unsigned int dim>
void MultivariateNormalDistribution<dim>::preComputeCovarianceMatrix()
{
	covarianceMatrixDeterminant = covarianceMatrix.determinant();
	inverseCovarianceMatrix = covarianceMatrix.inverse();
}

template<unsigned int dim>
void MultivariateNormalDistribution<dim>::setCovarianceMatrix(const Eigen::Matrix<double, dim, dim>& covarianceMatrix)
{
	this->covarianceMatrix = covarianceMatrix;
	preComputeCovarianceMatrix();
}

template<unsigned int dim>
double MultivariateNormalDistribution<dim>::operator()(const Eigen::Matrix<double, dim, 1>& x) const
{
	Eigen::Matrix<double, dim, 1> xMean = x - meanVector;
	double e = -0.5 * xMean.transpose() * inverseCovarianceMatrix * xMean;

	return 1 / std::sqrt(two_pi_k * covarianceMatrixDeterminant) * std::exp(e);
}

}}

#endif // LSL_PROBABILITY_MULTIVARIATENORMALDISTRIBUTION_HPP
