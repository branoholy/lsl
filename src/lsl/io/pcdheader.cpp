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

#include "lsl/io/pcdheader.hpp"

#include <algorithm>
#include <sstream>

namespace lsl {
namespace io {

std::string PCDHeader::NAME_VERSION = "VERSION";
std::string PCDHeader::NAME_FIELDS = "FIELDS";
std::string PCDHeader::NAME_SIZE = "SIZE";
std::string PCDHeader::NAME_TYPE = "TYPE";
std::string PCDHeader::NAME_COUNT = "COUNT";
std::string PCDHeader::NAME_WIDTH = "WIDTH";
std::string PCDHeader::NAME_HEIGHT = "HEIGHT";
std::string PCDHeader::NAME_VIEWPOINT = "VIEWPOINT";
std::string PCDHeader::NAME_POINTS = "POINTS";
std::string PCDHeader::NAME_DATA = "DATA";

PCDHeader::PCDHeader() :
	fieldCount(0), fields(nullptr), size(nullptr), type(nullptr), count(nullptr), width(0), height(0), points(0)
{
}

PCDHeader::PCDHeader(std::istream& stream) : PCDHeader()
{
	load(stream);
}

PCDHeader::PCDHeader(const PCDHeader& header) : PCDHeader()
{
	version = header.version;
	fieldCount = header.fieldCount;

	if(fieldCount > 0)
	{
		fields = new std::string[fieldCount];
		std::copy(header.fields, header.fields + fieldCount, fields);

		size = new std::size_t[fieldCount];
		std::copy(header.size, header.size + fieldCount, size);

		type = new char[fieldCount];
		std::copy(header.type, header.type + fieldCount, type);

		count = new std::size_t[fieldCount];
		std::copy(header.count, header.count + fieldCount, count);
	}

	width = header.width;
	height = header.height;
	viewpoint = header.viewpoint;
	points = header.points;
	data = header.data;
}

PCDHeader::~PCDHeader()
{
	delete[] fields;
	delete[] size;
	delete[] type;
	delete[] count;
}

void PCDHeader::load(std::istream& stream)
{
	std::string line;
	while(std::getline(stream, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;
		else
		{
			std::istringstream lineStream(line);

			std::string command;
			lineStream >> command;
			lineStream.get();

			if(command == NAME_VERSION) lineStream >> version;
			else if(command == NAME_FIELDS)
			{
				fieldCount = std::count(line.begin(), line.end(), ' ');

				fields = new std::string[fieldCount];
				loadFields(lineStream, fields);
			}
			else if(command == NAME_SIZE)
			{
				size = new std::size_t[fieldCount];
				loadFields(lineStream, size);
			}
			else if(command == NAME_TYPE)
			{
				type = new char[fieldCount];
				loadFields(lineStream, type);
			}
			else if(command == NAME_COUNT)
			{
				count = new std::size_t[fieldCount];
				loadFields(lineStream, count);
			}
			else if(command == NAME_WIDTH) lineStream >> width;
			else if(command == NAME_HEIGHT) lineStream >> height;
			else if(command == NAME_VIEWPOINT) getline(lineStream, viewpoint);
			else if(command == NAME_POINTS) lineStream >> points;
			else if(command == NAME_DATA)
			{
				lineStream >> data;
				break;
			}
		}
	}
}

void PCDHeader::save(std::ostream& stream)
{
	stream << NAME_VERSION << ' ' << version << std::endl;

	saveFields(stream, NAME_FIELDS, fields);
	saveFields(stream, NAME_SIZE, size);
	saveFields(stream, NAME_TYPE, type);
	saveFields(stream, NAME_COUNT, count);

	stream << NAME_WIDTH << ' ' << width << std::endl;
	stream << NAME_HEIGHT << ' ' << height << std::endl;
	stream << NAME_VIEWPOINT << ' ' << viewpoint << std::endl;
	stream << NAME_POINTS << ' ' << points << std::endl;
	stream << NAME_DATA << ' ' << data << std::endl;
}

}}
