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

#ifndef LSL_IO_PCDSTREAM_HPP
#define LSL_IO_PCDSTREAM_HPP

#include <fstream>
#include <sstream>
#include <vector>

#include "lsl/geom/vector.hpp"

#include "lsl/io/pcdheader.hpp"

namespace lsl {
namespace io {

class PCDStream
{
public:
	static PCDHeader* loadHeader(const std::string& fileName);

	template<typename OutputIterator>
	static void loadData(std::ifstream& file, std::size_t fieldCount, OutputIterator result);

	template<typename Container>
	static void loadDataEmplace(std::ifstream& file, std::size_t fieldCount, Container& container);

	template<typename InputIterator>
	static void saveData(std::ofstream& file, InputIterator begin, InputIterator end);
};

template<typename OutputIterator>
void PCDStream::loadData(std::ifstream& file, std::size_t fieldCount, OutputIterator result)
{
	typedef typename OutputIterator::container_type::value_type::value_type value_t;

	int i = 0;
	std::string line;
	value_t *data = new value_t[fieldCount + 1];

	while(getline(file, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;

		data[0] = i++;

		std::istringstream lineStream(line);
		for(std::size_t j = 1; j < fieldCount; j++)
		{
			lineStream >> data[j];
		}

		*result = PointType(data, fieldCount);
		result++;
	}

	delete[] data;
}

template<typename Container>
void PCDStream::loadDataEmplace(std::ifstream& file, std::size_t fieldCount, Container& container)
{
	typedef typename Container::value_type::value_type value_t;

	int i = 0;
	std::string line;
	fieldCount++;
	value_t *data = new value_t[fieldCount];

	while(getline(file, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;

		data[0] = i++;

		std::istringstream lineStream(line);
		for(std::size_t j = 1; j < fieldCount; j++)
		{
			lineStream >> data[j];
		}

		container.emplace_back(data, fieldCount);
	}

	delete[] data;
}

template<typename InputIterator>
void PCDStream::saveData(std::ofstream& file, InputIterator begin, InputIterator end)
{
	// TODO: Set precision based on header/point type.
	std::streamsize precision = file.precision();
	file.precision(8);
	while(begin != end)
	{
		file << *begin << std::endl;
		begin++;
	}
	file.precision(precision);
}

}}

#endif // LSL_IO_PCDSTREAM_HPP
