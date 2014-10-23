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

#include "lsl/registration/polarndt.hpp"

#include "lsl/optimization/soma.hpp"
#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/mathutils.hpp"

using namespace std;
using namespace lsl::geom;
using namespace lsl::optimization;
using namespace lsl::probability;
using namespace lsl::utils;

namespace lsl {
namespace registration {

PolarNDT::PolarNDT(size_t size) :
	size(size)
{
}

MultivariateNormalDistribution<2>** PolarNDT::createGaussians(const vector<Vector2d>& points) const
{
	vector<Vector2d> *vectors = new vector<Vector2d>[size];
	for(Vector2d p : points)
	{
		double theta = atan2(p.get(1), p.get(0)) + MathUtils::TWO_PI;
		int index = int(size * theta * MathUtils::ONE__TWO_PI) % size;
		vectors[index].push_back(p);
	}

	MultivariateNormalDistribution<2> **gaussians = ArrayUtils::createFilledArray<MultivariateNormalDistribution<2>*>(size, nullptr);
	for(size_t i = 0; i < size; i++)
	{
		if(!vectors[i].empty())
		{
			gaussians[i] = new MultivariateNormalDistribution<2>(vectors[i]);
		}
	}
	delete[] vectors;

	return gaussians;
}

double PolarNDT::errorTransform(MultivariateNormalDistribution<2> **gaussians, const vector<Vector2d>& source, double phi, double tx, double ty) const
{
	double c = cos(phi);
	double s = sin(phi);
	double error = 0;

	for(Vector2d p : source)
	{
		double x = p.get(0);
		double y = p.get(1);

		// Transform x, y
		double x_ = x * c - y * s + tx;
		double y_ = x * s + y * c + ty;

		double theta = atan2(y_, x_) + MathUtils::TWO_PI;
		int index = int(size * theta * MathUtils::ONE__TWO_PI) % size;

		if(gaussians[index] != nullptr) error += (*(gaussians[index]))({x_, y_});
	}

	return -error;
}

double PolarNDT::alignWithSOMA(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty) const
{
	const double specimen[][2] = {{phi - M_PI_2, phi + M_PI_2}, {tx - 300, tx + 300}, {ty - 200, ty + 200}};
	const double mean[] = {phi, tx, ty};
	const double sigmas[] = {2, 20, 20};

	// SOMA soma(3, 3, 0.1, 0.5, 30, 100, 0.001);
	SOMA soma(3, 2, 0.31, 0.8, 50, 1000, 0.00001);
	soma.setSpecimen(specimen);

	soma.initUniformPopulation();
	// soma.initNormalPopulation(mean, sigmas);

	MultivariateNormalDistribution<2> **gaussians = createGaussians(target);
	double error;

	double *opt = soma.minimize(
				[this, gaussians, &source](const double *transform) -> double
	{
		return this->errorTransform(gaussians, source, transform[0], transform[1], transform[2]);
	}
	, error);

	phi = opt[0];
	tx = opt[1];
	ty = opt[2];

	delete[] opt;
	ArrayUtils::deleteArray(gaussians, size);

	return error;
}

}}
