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

#ifndef LSL_IO_CARMENSTREAM_HPP
#define LSL_IO_CARMENSTREAM_HPP

#include <functional>
#include <limits>
#include <sstream>
#include <vector>

#include "lsl/io/carmencommand.hpp"
#include "lsl/io/stream.hpp"

#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace io {

template<typename PointCloudT>
class CARMENStream : public Stream<PointCloudT>
{
protected:
	double errorRange = std::numeric_limits<double>::max();
	std::streampos posg = 0;

	virtual void fillPointCloud(const CARMENCommandFLaser *commandFLaser, PointCloudT& pointCloud);

	virtual PointCloudT loadData(std::istream& stream);
	virtual void saveData(const PointCloudT& pointCloud, std::ostream& stream);

	void skipToTimestamp(double timestamp, std::size_t pos);

public:
	double zoom = 1;

	using Stream<PointCloudT>::Stream;

	CARMENStream(double errorRange);
	CARMENStream(const std::string& filePath, double errorRange);

	virtual PointCloudT load();
	virtual PointCloudT load(const std::string& filePath);
	virtual PointCloudT load(std::istream& stream);

	virtual std::vector<PointCloudT> loadAll(std::size_t n = -1);
	virtual std::vector<PointCloudT> loadAll(const std::string& filePath, std::size_t n = -1);
	virtual std::vector<PointCloudT> loadAll(std::istream& stream, std::size_t n = -1);

	bool tryLoadCommand(CARMENCommand& command, const std::string& onlyCmdName = "");
	bool tryLoadCommand(std::istream& stream, CARMENCommand& command, const std::string& onlyCmdName = "");

	bool tryLoadCommands(std::function<bool(CARMENCommand&)> func, const std::string& onlyCmdName = "");
	bool tryLoadCommands(std::istream& stream, std::function<bool(CARMENCommand&)> func, const std::string& onlyCmdName = "");

	void skipCommands(std::size_t n, const std::string& onlyCmdName = "");
	void skipToIpcTimestamp(double timestamp);
	void skipToLoggerTimestamp(double timestamp);
};

template<typename PointCloudT>
CARMENStream<PointCloudT>::CARMENStream(double errorRange) :
	errorRange(errorRange)
{
}

template<typename PointCloudT>
CARMENStream<PointCloudT>::CARMENStream(const std::string& filePath, double errorRange) : Stream<PointCloudT>(filePath),
	errorRange(errorRange)
{
}

template<typename PointCloudT>
PointCloudT CARMENStream<PointCloudT>::load()
{
	return load(this->filePath);
}

template<typename PointCloudT>
PointCloudT CARMENStream<PointCloudT>::load(const std::string& filePath)
{
	this->filePath = filePath;

	std::ifstream file(filePath);
	file.seekg(posg);

	return load(file);
}

template<typename PointCloudT>
PointCloudT CARMENStream<PointCloudT>::load(std::istream& stream)
{
	this->loaded = true;
	return loadData(stream);
}

template<typename PointCloudT>
std::vector<PointCloudT> CARMENStream<PointCloudT>::loadAll(std::size_t n)
{
	return loadAll(this->filePath, n);
}

template<typename PointCloudT>
std::vector<PointCloudT> CARMENStream<PointCloudT>::loadAll(const std::string& filePath, std::size_t n)
{
	this->filePath = filePath;

	std::ifstream file(filePath);
	file.seekg(posg);

	return loadAll(file, n);
}

template<typename PointCloudT>
std::vector<PointCloudT> CARMENStream<PointCloudT>::loadAll(std::istream& stream, std::size_t n)
{
	std::vector<PointCloudT> pointClouds;

	if(n > 0)
	{
		this->loaded = true;

		std::size_t i = 0;
		tryLoadCommands(stream, [this, &pointClouds, &i, n] (CARMENCommand& command)
		{
			pointClouds.emplace_back();
			pointClouds.back().setId(command.id);
			fillPointCloud(command.flaser(), pointClouds.back());

			return (++i < n);
		}, CARMENCommand::CMD_FLASER);
	}

	return pointClouds;
}

template<typename PointCloudT>
bool CARMENStream<PointCloudT>::tryLoadCommand(CARMENCommand& command, const std::string& onlyCmdName)
{
	std::ifstream file(this->filePath);
	file.seekg(posg);

	bool result = tryLoadCommand(file, command, onlyCmdName);
	posg = file.tellg();

	return result;
}

template<typename PointCloudT>
bool CARMENStream<PointCloudT>::tryLoadCommand(std::istream& stream, CARMENCommand& command, const std::string& onlyCmdName)
{
	std::string line;
	while(std::getline(stream, line))
	{
		if(command.parse(line, onlyCmdName)) return true;
	}

	return false;
}

template<typename PointCloudT>
bool CARMENStream<PointCloudT>::tryLoadCommands(std::function<bool(CARMENCommand&)> func, const std::string& onlyCmdName)
{
	std::ifstream file(this->filePath);
	file.seekg(posg);

	bool result = tryLoadCommands(file, func, onlyCmdName);
	posg = file.tellg();

	return result;
}

template<typename PointCloudT>
bool CARMENStream<PointCloudT>::tryLoadCommands(std::istream& stream, std::function<bool(CARMENCommand&)> func, const std::string& onlyCmdName)
{
	std::string line;
	while(std::getline(stream, line))
	{
		CARMENCommand command;
		if(command.parse(line, onlyCmdName) && !func(command)) return true;
	}

	return false;
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::fillPointCloud(const CARMENCommandFLaser *commandFLaser, PointCloudT& pointCloud)
{
	if(PointCloudT::Point::RowsAtCompileTime != 3)
	{
		throw std::ios_base::failure("CARMEN supports 2D scans only.");
	}

	typename PointCloudT::ScalarType data[PointCloudT::Point::RowsAtCompileTime];

	pointCloud.getOdomLocation() << zoom * commandFLaser->x, zoom * commandFLaser->y, commandFLaser->theta;
	pointCloud.reserve(commandFLaser->numReadings);

	double deltaAngle = utils::MathUtils::PI / commandFLaser->numReadings;
	for(std::size_t i = 0; i < commandFLaser->numReadings; i++)
	{
		double range = commandFLaser->rangeReadings[i];
		if(range >= errorRange) continue;

		range *= zoom;
		double angle = i * deltaAngle - utils::MathUtils::PI__TWO;

		data[0] = range * std::cos(angle);
		data[1] = range * std::sin(angle);
		data[2] = 1;

		pointCloud.emplace_back(data);

		typename PointCloudT::Point& point = pointCloud.back();
		point.setId(i);
		point.realPoint = true;
	}
}

template<typename PointCloudT>
PointCloudT CARMENStream<PointCloudT>::loadData(std::istream& stream)
{
	PointCloudT pointCloud;

	CARMENCommand command;
	if(tryLoadCommand(stream, command, CARMENCommand::CMD_FLASER))
	{
		pointCloud.setId(command.id);
		fillPointCloud(command.flaser(), pointCloud);
	}

	return pointCloud;
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::saveData(const PointCloudT&, std::ostream&)
{
	// TODO: Implement saving into CARMEN-logfiles.
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::skipCommands(std::size_t n, const std::string& onlyCmdName)
{
	std::size_t i = 0;

	std::ifstream file(this->filePath);
	file.seekg(posg);

	std::string line;
	while(i < n && std::getline(file, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;

		std::istringstream lineStream(line);
		std::string cmdName;
		lineStream >> cmdName;

		if(onlyCmdName.empty() || cmdName == onlyCmdName) i++;
	}

	posg = file.tellg();
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::skipToTimestamp(double timestamp, std::size_t pos)
{
	double ipcTimestamp = 0;

	std::ifstream file(this->filePath);
	file.seekg(posg);

	std::string line;
	while(file)
	{
		posg = file.tellg();
		std::getline(file, line);

		if(line.empty()) continue;
		if(line[0] == '#') continue;

		std::size_t startPos = std::string::npos, endPos = std::string::npos;
		for(std::size_t i = 0; i < pos; i++)
		{
			endPos = startPos;
			startPos = line.rfind(' ', startPos - 1);
		}

		ipcTimestamp = std::stod(line.substr(startPos + 1, endPos - startPos - 1));
		if(ipcTimestamp >= timestamp) break;
	}
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::skipToIpcTimestamp(double timestamp)
{
	skipToTimestamp(timestamp, 3);
}

template<typename PointCloudT>
void CARMENStream<PointCloudT>::skipToLoggerTimestamp(double timestamp)
{
	skipToTimestamp(timestamp, 1);
}

}}

#endif // LSL_IO_CARMENSTREAM_HPP
