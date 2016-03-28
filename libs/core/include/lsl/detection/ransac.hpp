/*
 * LIDAR System Library
 * Copyright (C) 2014-2016  Branislav Holý <branoholy@gmail.com>
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

#ifndef LSL_DETECTION_RANSAC_HPP
#define LSL_DETECTION_RANSAC_HPP

//#include <algorithm>
#include <limits>
#include <random>
#include <set>

#include "lsl/geom/line2.hpp"

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/cpputils.hpp"

namespace lsl {
namespace detection {

class Ransac
{
private:
	static std::mt19937 rnd;

	std::size_t iterations;
	std::size_t initModelSize;
	std::size_t minModelSize;
	double maxError;

public:
	Ransac();
	Ransac(std::size_t iterations, std::size_t initModelSize, std::size_t minModelSize, double maxError);

	void set(std::size_t iterations, std::size_t initModelSize, std::size_t minModelSize, double maxError);

	template<typename T>
	std::vector<T> run(const std::vector<T>& points) const;

	template<typename R, typename T>
	std::vector<R> run(const std::vector<T>& points, std::size_t outterIterations) const;

	static std::size_t getRandom(std::size_t max);
};

template<typename T>
std::vector<T> Ransac::run(const std::vector<T>& points) const
{
	typedef typename std::remove_pointer<T>::type PointType;
	typedef typename std::conditional<std::is_pointer<T>::value, PointType*, const PointType*>::type ModelValueType;

	std::set<ModelValueType> modelData;
	std::vector<T> bestModelData;

	double bestError2 = std::numeric_limits<double>::max();
	double maxError2 = maxError * maxError;

	for(std::size_t i = 0; i < iterations; i++)
	{
		modelData.clear();
		std::size_t tries = 0;
		while(modelData.size() < initModelSize && tries < initModelSize * 10)
		{
			const T& point = points.at(getRandom(points.size()));
			modelData.insert(utils::CppUtils::getPointer(point));
			tries++;
		}

		geom::Line2 model = geom::Line2::leastSquareLine(modelData, false);

		modelData.clear();
		double error2 = 0;
		ModelValueType lastPointPointer = nullptr;
		for(std::size_t j = 0; j < points.size(); j++)
		{
			const T& point = points.at(j);
			ModelValueType pointPointer = utils::CppUtils::getPointer(point);
			double distance2 = model.distance2To(*pointPointer);

			bool useLastPoint = (lastPointPointer == nullptr || ((pointPointer->getId() == lastPointPointer->getId() + 1) && (*pointPointer - *lastPointPointer).squaredNorm() < 100 * maxError2));
			if(distance2 < maxError2 && useLastPoint)
			{
				modelData.insert(pointPointer);
				error2 += distance2;
				lastPointPointer = pointPointer;
			}
			else
			{
				if(modelData.size() >= minModelSize) break;
				else
				{
					modelData.clear();
					error2 = 0;
				}
			}
		}

		if(modelData.size() >= minModelSize)
		{
			error2 /= modelData.size();
			if(error2 < bestError2)
			{
				bool testOk = true;
				geom::Line2 testModel = geom::Line2::leastSquareLine(modelData, false);
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

template<typename R, typename T>
std::vector<R> Ransac::run(const std::vector<T>& points, std::size_t outterIterations) const
{
	std::vector<R> allModelData;
	std::vector<T> remainingPoints = points;

	for(std::size_t i = 0; i < outterIterations; i++)
	{
		if(remainingPoints.size() == 0) break;

		std::vector<T> modelData = run(remainingPoints);
		if(modelData.size() > 0)
		{
			try
			{
				allModelData.emplace_back(modelData);
				utils::ArrayUtils::eraseAll(remainingPoints, modelData.begin(), modelData.end());
			}
			catch(const std::invalid_argument&)
			{
			}
		}
	}

	return allModelData;
}

}}

#endif // LSL_DETECTION_RANSAC_HPP
