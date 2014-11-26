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

#ifndef LSL_OPTIMIZATION_SOMA_HPP
#define LSL_OPTIMIZATION_SOMA_HPP

#include <cstddef>
#include <functional>

namespace lsl {
namespace optimization {

class SOMA
{
private:
	typedef struct { double low, high; } Speciman;

private:
	const std::size_t dimension;
	const std::size_t oi;
	double mass;
	double step;
	double prt;
	std::size_t np;
	std::size_t migration;
	double acceptedError;

	Speciman *specimen;
	double **population;

	std::size_t evals;
	bool converged;

public:
	SOMA(std::size_t dimension, double mass, double step, double prt, std::size_t np, std::size_t migration, double acceptedError);
	~SOMA();

	inline std::size_t getEvaluationCount() const { return evals; }
	inline bool hasConverged() const { return converged; }

	double* minimize(std::function<double(const double*)> errorFunction, double& output);

	void setSpecimen(std::size_t dimension, double low, double high);
	void setSpecimen(const double specimen[][2]);

	void initUniformPopulation();
	void initNormalPopulation(const double *mean, const double *sigmas);
};

}}

#endif // LSL_OPTIMIZATION_SOMA_HPP
