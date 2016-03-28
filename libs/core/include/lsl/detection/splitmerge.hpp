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

#ifndef LSL_DETECTION_SPLITMERGE_HPP
#define LSL_DETECTION_SPLITMERGE_HPP

#include <limits>
#include <vector>

#include "lsl/geom/line2.hpp"

#include "lsl/utils/cpputils.hpp"

namespace lsl {
namespace detection {

class SplitMerge
{
private:
	std::size_t minModelSize;
	double maxError;

	template<typename T>
	void run(const std::vector<T>& points, std::size_t start, std::size_t end, std::vector<std::size_t>& indices, std::vector<std::size_t>& lists) const;

public:
	SplitMerge();
	SplitMerge(std::size_t minModelSize, double maxError);

	void set(std::size_t minModelSize, double maxError);

	template<typename R, typename T>
	std::vector<R> run(std::vector<T> points) const;

	template<typename R, typename T>
	std::vector<R> run(std::vector<T> points, std::vector<T>& remainingPoints) const;

	template<typename T>
	std::vector<std::size_t> getLists(const std::vector<T>& points) const;
};

template<typename T>
void SplitMerge::run(const std::vector<T>& points, std::size_t start, std::size_t end, std::vector<std::size_t>& indices, std::vector<std::size_t>& lists) const
{
	if((end - start) >= minModelSize)
	{
		typedef typename std::remove_pointer<T>::type PointType;
		typedef typename std::conditional<std::is_pointer<T>::value, PointType*, const PointType*>::type ModelValueType;

		lists.push_back(start);
		lists.push_back(end);

		geom::Line2 model(points.at(start), points.at(end - 1), false);

		size_t i = start;
		int worstId = -1;
		double worstError2 = maxError * maxError;

		typename std::vector<T>::const_iterator endIt = points.begin() + end;
		for(auto it = points.begin() + start; it != endIt; it++, i++)
		{
			const T& point = *it;
			ModelValueType pointPointer = utils::CppUtils::getPointer(point);
			double distance2 = model.distance2To(*pointPointer);

			if(distance2 > worstError2)
			{
				worstError2 = distance2;
				worstId = i;
			}
		}

		lists.push_back(worstId);
		if(worstId >= 0)
		{
			lists.push_back(-2);

			run(points, start, worstId, indices, lists);
			run(points, worstId + 1, end, indices, lists);
		}
		else
		{
			lists.push_back(-1);

			indices.push_back(start);
			indices.push_back(end);
		}
	}
}

template<typename R, typename T>
std::vector<R> SplitMerge::run(std::vector<T> points) const
{
	std::vector<T> remainingPoints;
	return run(points, remainingPoints);
}

template<typename R, typename T>
std::vector<R> SplitMerge::run(std::vector<T> points, std::vector<T>& remainingPoints) const
{
	std::vector<R> allModelData;

	while(true)
	{
		std::vector<std::size_t> indices, lists;
		run(points, 0, points.size(), indices, lists);

		if(indices.empty()) break;

		for(std::size_t i = 0; i < indices.size(); i += 2)
		{
			std::vector<T> modelData(points.begin() + indices.at(i), points.begin() + indices.at(i + 1));

			try
			{
				allModelData.emplace_back(modelData);
			}
			catch(const std::invalid_argument&)
			{
			}
		}

		remainingPoints.clear();
		std::size_t start = 0, end = 0;
		for(std::size_t i = 0; i < indices.size(); i += 2)
		{
			end = indices.at(i);
			if(start < end)
			{
				remainingPoints.insert(remainingPoints.end(), points.begin() + start, points.begin() + end);
			}
			start = indices.at(i + 1);
		}

		points = remainingPoints;
	}

	return allModelData;
}

template<typename T>
std::vector<std::size_t> SplitMerge::getLists(const std::vector<T>& points) const
{
	std::vector<std::size_t> indices, lists;
	run(points, 0, points.size(), indices, lists);

	return lists;
}

}}

#endif // LSL_DETECTION_SPLITMERGE_HPP
