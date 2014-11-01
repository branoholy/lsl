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

#ifndef LSL_GEOM_RANSAC_HPP
#define LSL_GEOM_RANSAC_HPP

#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <vector>

#include "line2.hpp"
#include "vector.hpp"

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/cpputils.hpp"

namespace lsl {
namespace geom {

class Ransac
{
private:
	std::mt19937 rnd;

	int iterations;
	int initModelSize;
	int minModelSize;
	double maxError;

	int getRandom(int max);

public:
	Ransac(int iterations, int initModelSize, int minModelSize, double maxError);

	template<typename T>
	std::vector<T> run(const std::vector<T>& points);

	template<typename T>
	std::vector<std::vector<T>> run(const std::vector<T>& points, unsigned int outterIterations);
};

template<typename T>
std::vector<T> Ransac::run(const std::vector<T>& points)
{
	std::set<T> modelData;
	std::vector<T> bestModelData;
	double bestError2 = std::numeric_limits<double>::max();
	double maxError2 = maxError * maxError;

	for(int i = 0; i < iterations; i++)
	{
		modelData.clear();
		int tries = 0;
		while(modelData.size() < initModelSize && tries < initModelSize * 10)
		{
			modelData.insert(points.at(getRandom(points.size())));
			tries++;
		}

		Line2 model = Line2::leastSquareLine(modelData);

		double error2 = 0;
		for(int j = 0; j < points.size(); j++)
		{
			T point = points.at(j);
			double distance2 = model.distance2To(*utils::CppUtils::getPointer(point));

			if(distance2 < maxError2)
			{
				modelData.insert(point);
				error2 += distance2;
			}
			else
			{
				error2 += maxError2;
			}
		}

		if(modelData.size() >= minModelSize)
		{
			if(error2 < bestError2)
			{
				bestModelData.clear();
				std::copy(modelData.begin(), modelData.end(), back_inserter(bestModelData));
				bestError2 = error2;
			}
		}
	}

	return bestModelData;
}

template<typename T>
std::vector<std::vector<T>> Ransac::run(const std::vector<T>& points, unsigned int outterIterations)
{
	std::vector<std::vector<T>> allModelData;
	std::vector<T> remainingPoints = points;

	for(unsigned int i = 0; i < outterIterations; i++)
	{
		if(remainingPoints.size() == 0) break;

		std::vector<T> modelData = run(remainingPoints);
		if(modelData.size() > 0)
		{
			allModelData.push_back(modelData);
			utils::ArrayUtils::eraseAll(remainingPoints, modelData.begin(), modelData.end());
		}
	}

	return allModelData;
}

}}

#endif // LSL_GEOM_RANSAC_HPP
