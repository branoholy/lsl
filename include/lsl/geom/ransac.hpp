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

#include <algorithm>
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
	typedef typename std::remove_pointer<T>::type PointType;
	typedef typename std::conditional<std::is_pointer<T>::value, PointType*, const PointType*>::type ModelValueType;

	std::set<ModelValueType> modelData;
	std::vector<T> bestModelData;

	double bestError2 = std::numeric_limits<double>::max();
	double maxError2 = maxError * maxError;

	for(int i = 0; i < iterations; i++)
	{
		modelData.clear();
		int tries = 0;
		while(modelData.size() < initModelSize && tries < initModelSize * 10)
		{
			const T& point = points.at(getRandom(points.size()));
			modelData.insert(utils::CppUtils::getPointer(point));
			tries++;
		}

		Line2 model = Line2::leastSquareLine(modelData, false);

		modelData.clear();
		double error2 = 0;
		unsigned int lastPointId = -1;
		for(int j = 0; j < points.size(); j++)
		{
			const T& point = points.at(j);
			ModelValueType pointPointer = utils::CppUtils::getPointer(point);
			double distance2 = model.distance2To(*pointPointer);

			if(distance2 < maxError2 && (lastPointId == -1 || pointPointer->getId() == lastPointId + 1))
			{
				modelData.insert(pointPointer);
				error2 += distance2;
				lastPointId = pointPointer->getId();
			}
			else
			{
				if(modelData.size() >= minModelSize) break;
				else
				{
					modelData.clear();
					error2 = 0;
				}

				// error2 += maxError2;
			}
		}

		if(modelData.size() >= minModelSize)
		{
			error2 /= modelData.size();
			if(error2 < bestError2)
			{
				bool testOk = true;
				Line2 testModel = Line2::leastSquareLine(modelData, false);
				for(const ModelValueType& testPoint : modelData)
				{
					if(testModel.distance2To(*testPoint) > maxError2)
					{
						testOk = false;
						break;
					}
				}

				if(testOk)
				{
					bestModelData.clear();
					bestModelData.reserve(modelData.size());
					std::transform(modelData.begin(), modelData.end(), back_inserter(bestModelData), utils::CppUtils::getValue<T, ModelValueType>);

					bestError2 = error2;
				}
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
