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

#ifndef LSL_CONTAINERS_POINTCLOUD_HPP
#define LSL_CONTAINERS_POINTCLOUD_HPP

#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

#include "lsl/utils/arrayutils.hpp"
#include "lsl/io/pcdheader.hpp"
#include "lsl/io/pcdstream.hpp"

namespace lsl {
namespace containers {

template<typename T>
class PointCloud : public std::vector<T>
{
private:
	io::PCDHeader *header;

	void correctIds();

public:
	PointCloud();
	PointCloud(const std::string& fileName);
	~PointCloud();

	inline io::PCDHeader* getHeader() const { return header; }

	void loadPCD(const std::string& fileName);
	void savePCD(const std::string& fileName);

	template<typename T_>
	friend std::ostream& operator<<(std::ostream& out, const PointCloud<T_>& pointCloud);
};

template<typename T>
PointCloud<T>::PointCloud() :
	header(nullptr)
{
}

template<typename T>
PointCloud<T>::PointCloud(const std::string& fileName) : PointCloud()
{
	loadPCD(fileName);
}

template<typename T>
PointCloud<T>::~PointCloud()
{
	delete header;
}

template<typename T>
void PointCloud<T>::correctIds()
{
	std::sort(this->begin(), this->end(), [](const T& a, const T& b) { return a.getAngle2D() < b.getAngle2D(); });

	size_t size = this->size();
	for(size_t i = 0; i < size; i++)
	{
		(*this)[i].setId(i);
	}
}

template<typename T>
void PointCloud<T>::loadPCD(const std::string& fileName)
{
	std::ifstream pcdFile(fileName);

	header = new io::PCDHeader(pcdFile);
	this->reserve(header->points);

	io::PCDStream::loadDataEmplace(pcdFile, header->fieldCount, *this);
	correctIds();
}

template<typename T>
void PointCloud<T>::savePCD(const std::string& fileName)
{
	if(header != nullptr)
	{
		std::ofstream pcdFile(fileName);

		header->save(pcdFile);
		io::PCDStream::saveData(pcdFile, this->cbegin(), this->cend());
	}
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const PointCloud<T>& pointCloud)
{
	using ::operator<<;
	return out << static_cast<const std::vector<T>&>(pointCloud);
}

}}

#endif // LSL_CONTAINERS_POINTCLOUD_HPP
