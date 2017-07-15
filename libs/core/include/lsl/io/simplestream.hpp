/*
 * LIDAR System Library
 * Copyright (C) 2014-2017  Branislav Holý <branoholy@gmail.com>
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

#ifndef LSL_IO_SIMPLESTREAM_HPP
#define LSL_IO_SIMPLESTREAM_HPP

#include <limits>
#include <sstream>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "lsl/io/stream.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace io {

template<typename PointCloudT>
class SimpleStream : public Stream<PointCloudT>
{
protected:
	double errorRange = std::numeric_limits<double>::max();

	virtual PointCloudT loadData(std::istream& stream);
	virtual void saveData(const PointCloudT& pointCloud, std::ostream& stream);

public:
	double zoom = 1;

	using Stream<PointCloudT>::Stream;

	SimpleStream(double errorRange);
	SimpleStream(const std::string& filePath, double errorRange);

	virtual std::vector<PointCloudT> loadAll(std::size_t n = -1);
	virtual std::vector<PointCloudT> loadAll(const std::string& filePath, std::size_t n = -1);
	virtual std::vector<PointCloudT> loadAll(std::istream& stream, std::size_t n = -1);
};

template<typename PointCloudT>
SimpleStream<PointCloudT>::SimpleStream(double errorRange) :
	errorRange(errorRange)
{
}

template<typename PointCloudT>
SimpleStream<PointCloudT>::SimpleStream(const std::string& filePath, double errorRange) : Stream<PointCloudT>(filePath),
	errorRange(errorRange)
{
}

template<typename PointCloudT>
std::vector<PointCloudT> SimpleStream<PointCloudT>::loadAll(std::size_t n)
{
	return loadAll(this->filePath, n);
}

template<typename PointCloudT>
std::vector<PointCloudT> SimpleStream<PointCloudT>::loadAll(const std::string& filePath, std::size_t n)
{
	this->filePath = filePath;

	std::ifstream file(filePath);
	return loadAll(file, n);
}

template<typename PointCloudT>
std::vector<PointCloudT> SimpleStream<PointCloudT>::loadAll(std::istream& stream, std::size_t n)
{
	std::vector<PointCloudT> pointClouds;

	if(n > 0)
	{
		this->loaded = true;

		for(std::size_t id = 0; id < n; id++)
		{
			PointCloudT pointCloud = loadData(stream);
			pointCloud.setId(id);

			pointClouds.push_back(pointCloud);
		}
	}

	return pointClouds;
}

template<typename PointCloudT>
PointCloudT SimpleStream<PointCloudT>::loadData(std::istream& stream)
{
	if(PointCloudT::Point::RowsAtCompileTime != 3)
	{
		throw std::ios_base::failure("SimpleStream supports 2D scans only.");
	}

	typename PointCloudT::ScalarType data[PointCloudT::Point::RowsAtCompileTime];

	PointCloudT pointCloud;

	std::string line;
	while(std::getline(stream, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;

		std::istringstream lineStream(line);

		std::string command;
		lineStream >> command;
		boost::algorithm::to_lower(command);

		if(command == "odometry")
		{
			double x, y, theta;
			lineStream >> x >> y >> theta;

			pointCloud.getOdomLocation() << zoom * x, zoom * y, theta;
		}
		else if(command == "laser")
		{
			std::size_t numReadings;
			lineStream >> numReadings;
			pointCloud.reserve(numReadings);

			for(std::size_t id = 0; id < numReadings; id++)
			{
				double range;
				lineStream >> range;

				if(range >= errorRange) continue;

				range *= zoom;
				double angle = utils::MathUtils::toRadians(int(id) - 90);

				data[0] = range * std::cos(angle);
				data[1] = range * std::sin(angle);
				data[2] = 1;

				pointCloud.emplace_back(data);

				auto& point = pointCloud.back();
				point.setId(id);
				point.realPoint = true;
			}

			break;
		}
	}

	return pointCloud;
}

template<typename PointCloudT>
void SimpleStream<PointCloudT>::saveData(const PointCloudT&, std::ostream&)
{
	// TODO: Implement saving into SimpleStream-logfiles.
}

}}

#endif // LSL_IO_SIMPLESTREAM_HPP
