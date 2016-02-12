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

#ifndef LSL_IO_STREAM_HPP
#define LSL_IO_STREAM_HPP

#include <iostream>
#include <fstream>

namespace lsl {
namespace io {

template<typename PointCloudT>
class Stream
{
protected:
	std::string filePath;
	bool loaded;

	virtual PointCloudT loadData(std::istream& stream) = 0;
	virtual void saveData(const PointCloudT& pointCloud, std::ostream& stream) = 0;

public:
	Stream();
	Stream(const std::string& filePath);
	virtual ~Stream();

	inline const std::string& getFilePath() const { return filePath; }
	inline void setFilePaht(const std::string& filePath) { this->filePath = filePath; }

	inline bool isLoaded() const { return loaded; }

	virtual PointCloudT load();
	virtual PointCloudT load(const std::string& filePath);
	virtual PointCloudT load(std::istream& stream);

	virtual void save(const PointCloudT& pointCloud);
	virtual void save(const PointCloudT& pointCloud, const std::string& filePath);
	virtual void save(const PointCloudT& pointCloud, std::ostream& stream);
};

template<typename PointCloudT>
Stream<PointCloudT>::Stream() :
	loaded(false)
{
}

template<typename PointCloudT>
Stream<PointCloudT>::Stream(const std::string& filePath) :
	filePath(filePath), loaded(false)
{
}

template<typename PointCloudT>
Stream<PointCloudT>::~Stream()
{
}

template<typename PointCloudT>
PointCloudT Stream<PointCloudT>::load()
{
	return load(filePath);
}

template<typename PointCloudT>
PointCloudT Stream<PointCloudT>::load(const std::string& filePath)
{
	this->filePath = filePath;

	std::ifstream file(filePath);
	return load(file);
}

template<typename PointCloudT>
PointCloudT Stream<PointCloudT>::load(std::istream& stream)
{
	loaded = true;
	return loadData(stream);
}

template<typename PointCloudT>
void Stream<PointCloudT>::save(const PointCloudT& pointCloud)
{
	save(pointCloud, filePath);
}

template<typename PointCloudT>
void Stream<PointCloudT>::save(const PointCloudT& pointCloud, const std::string& filePath)
{
	this->filePath = filePath;

	std::ofstream file(filePath);
	save(pointCloud, file);
}

template<typename PointCloudT>
void Stream<PointCloudT>::save(const PointCloudT& pointCloud, std::ostream& stream)
{
	saveData(pointCloud, stream);
}

}}

#endif // LSL_IO_STREAM_HPP
