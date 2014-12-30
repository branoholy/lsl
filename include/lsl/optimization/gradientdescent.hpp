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

#ifndef LSL_OPTIMIZATION_GRADIENTDESCENT_HPP
#define LSL_OPTIMIZATION_GRADIENTDESCENT_HPP

#include <functional>

namespace lsl {
namespace optimization {

class GradientDescent
{
private:
	const std::size_t dimension;
	double *deltas;
	double *gammas;
	double precision;

	std::size_t evals;
	bool converged;

	double inputDistance(const double *input1, const double *input2) const;

public:
	GradientDescent(std::size_t dimension, double *deltas, double *gammas, double precision);

	inline std::size_t getEvaluationCount() const { return evals; }
	inline bool hasConverged() const { return converged; }

	double* minimize(std::function<double(const double*)> errorFunction, const double *guess, double& output);
	double* minimizeByDimension(std::function<double(const double*)> errorFunction, const double *guess, double& output);
};

}}

#endif // LSL_OPTIMIZATION_GRADIENTDESCENT_HPP
