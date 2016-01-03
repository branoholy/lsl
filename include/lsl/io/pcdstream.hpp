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

#ifndef LSL_IO_PCDSTREAM_HPP
#define LSL_IO_PCDSTREAM_HPP

#include <sstream>

#include "lsl/io/pcdheader.hpp"
#include "lsl/io/stream.hpp"

namespace lsl {
namespace io {

template<typename PointCloudT>
class PCDStream : public Stream<PointCloudT>
{
protected:
	PCDHeader *header;

	virtual PointCloudT loadData(std::istream& stream);
	virtual void saveData(const PointCloudT& pointCloud, std::ostream& stream);

public:
	PCDStream();
	PCDStream(const std::string& filePath);
	virtual ~PCDStream();

	inline PCDHeader* getHeader() { return header; }
	PCDHeader* loadHeader(const std::string& filePath);
	PCDHeader* loadHeader(std::istream& stream);
	PCDHeader* loadHeader(const PointCloudT& pointCloud);
};

template<typename PointCloudT>
PCDStream<PointCloudT>::PCDStream() : Stream<PointCloudT>(),
	header(nullptr)
{
}

template<typename PointCloudT>
PCDStream<PointCloudT>::PCDStream(const std::string& filePath) : Stream<PointCloudT>(filePath),
	header(nullptr)
{
}

template<typename PointCloudT>
PCDStream<PointCloudT>::~PCDStream()
{
	delete header;
}

template<typename PointCloudT>
PCDHeader* PCDStream<PointCloudT>::loadHeader(const std::string& filePath)
{
	std::ifstream file(filePath);
	return loadHeader(file);
}

template<typename PointCloudT>
PCDHeader* PCDStream<PointCloudT>::loadHeader(std::istream& stream)
{
	header = new PCDHeader(stream);
	return header;
}

template<typename PointCloudT>
PCDHeader* PCDStream<PointCloudT>::loadHeader(const PointCloudT& pointCloud)
{
	typedef typename PointCloudT::ScalarType ScalarType;

	std::size_t size = sizeof(ScalarType);

	char type = '-';
	if(std::is_integral<ScalarType>::value)
	{
		if(std::is_signed<ScalarType>::value) type = 'I';
		else if(std::is_unsigned<ScalarType>::value) type = 'U';
	}
	else if(std::is_floating_point<ScalarType>::value) type = 'F';

	delete[] header;
	header = new PCDHeader();

	header->version = "0.7";
	header->fieldCount = PointCloudT::dimension;

	header->fields = new std::string[header->fieldCount];
	header->size = new std::size_t[header->fieldCount];
	header->type = new char[header->fieldCount];
	header->count = new std::size_t[header->fieldCount];

	for(std::size_t d = 0; d < header->fieldCount; d++)
	{
		header->fields[d] = 'x' + d;
		header->size[d] = size;
		header->type[d] = type;
		header->count[d] = 1;
	}

	header->width = pointCloud.size();
	header->height = 1;
	header->viewpoint = "0 0 0 1 0 0 0";
	header->points = pointCloud.size();

	header->data = "ascii";

	return header;
}

template<typename PointCloudT>
PointCloudT PCDStream<PointCloudT>::loadData(std::istream& stream)
{
	delete header;
	loadHeader(stream);

	int id = 0;

	if(PointCloudT::Point::RowsAtCompileTime != header->fieldCount + 1)
	{
		throw std::ios_base::failure("Field count does not match.");
	}

	typename PointCloudT::ScalarType data[PointCloudT::Point::RowsAtCompileTime];

	PointCloudT pointCloud;
	pointCloud.reserve(header->points);

	std::string line;
	while(std::getline(stream, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;

		std::istringstream lineStream(line);
		for(std::size_t i = 0; i < header->fieldCount; i++)
		{
			lineStream >> data[i];
		}
		data[header->fieldCount] = 1;

		pointCloud.emplace_back(data);

		auto& point = pointCloud.back();

		point.setId(id++);
		point.realPoint = true;
	}

	return pointCloud;
}

template<typename PointCloudT>
void PCDStream<PointCloudT>::saveData(const PointCloudT& pointCloud, std::ostream& stream)
{
	loadHeader(pointCloud);
	header->save(stream);

	// TODO: Set precision based on header/point type.
	std::streamsize precision = stream.precision();
	stream.precision(8);

	for(const auto& point : pointCloud)
	{
		stream << point[0];
		for(std::size_t i = 1; i < header->fieldCount; i++)
		{
			stream << ' ' << point[i];
		}
		stream << std::endl;
	}

	stream.precision(precision);
}

}}

#endif // LSL_IO_PCDSTREAM_HPP
