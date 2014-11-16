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

#include "lsl/io/pcdheader.hpp"

#include <algorithm>
#include <sstream>

using namespace std;

namespace lsl {
namespace io {

string PCDHeader::NAME_VERSION = "VERSION";
string PCDHeader::NAME_FIELDS = "FIELDS";
string PCDHeader::NAME_SIZE = "SIZE";
string PCDHeader::NAME_TYPE = "TYPE";
string PCDHeader::NAME_COUNT = "COUNT";
string PCDHeader::NAME_WIDTH = "WIDTH";
string PCDHeader::NAME_HEIGHT = "HEIGHT";
string PCDHeader::NAME_VIEWPOINT = "VIEWPOINT";
string PCDHeader::NAME_POINTS = "POINTS";
string PCDHeader::NAME_DATA = "DATA";

PCDHeader::PCDHeader() :
	fieldCount(0), fields(nullptr), size(nullptr), type(nullptr), count(nullptr), width(0), height(0), points(0)
{
}

PCDHeader::PCDHeader(ifstream& file) : PCDHeader()
{
	load(file);
}

PCDHeader::~PCDHeader()
{
	delete[] fields;
	delete[] size;
	delete[] type;
	delete[] count;
}

void PCDHeader::load(ifstream& file)
{
	string line;
	while(getline(file, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;
		else
		{
			istringstream lineStream(line);

			string command;
			lineStream >> command;
			lineStream.get();

			if(command == NAME_VERSION) lineStream >> version;
			else if(command == NAME_FIELDS)
			{
				fieldCount = std::count(line.begin(), line.end(), ' ');

				fields = new string[fieldCount];
				loadFields(lineStream, fields);
			}
			else if(command == NAME_SIZE)
			{
				size = new unsigned int[fieldCount];
				loadFields(lineStream, size);
			}
			else if(command == NAME_TYPE)
			{
				type = new char[fieldCount];
				loadFields(lineStream, type);
			}
			else if(command == NAME_COUNT)
			{
				count = new unsigned int[fieldCount];
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

void PCDHeader::save(ofstream& file)
{
	file << NAME_VERSION << ' ' << version << endl;

	saveFields(file, NAME_FIELDS, fields);
	saveFields(file, NAME_SIZE, size);
	saveFields(file, NAME_TYPE, type);
	saveFields(file, NAME_COUNT, count);

	file << NAME_WIDTH << ' ' << width << endl;
	file << NAME_HEIGHT << ' ' << height << endl;
	file << NAME_VIEWPOINT << ' ' << viewpoint << endl;
	file << NAME_POINTS << ' ' << points << endl;
	file << NAME_DATA << ' ' << data << endl;
}

}}
