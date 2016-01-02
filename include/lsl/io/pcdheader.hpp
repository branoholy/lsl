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

#ifndef LSL_IO_PCDHEADER_HPP
#define LSL_IO_PCDHEADER_HPP

#include <iostream>

namespace lsl {
namespace io {

class PCDHeader
{
private:
	static std::string NAME_VERSION;
	static std::string NAME_FIELDS;
	static std::string NAME_SIZE;
	static std::string NAME_TYPE;
	static std::string NAME_COUNT;
	static std::string NAME_WIDTH;
	static std::string NAME_HEIGHT;
	static std::string NAME_VIEWPOINT;
	static std::string NAME_POINTS;
	static std::string NAME_DATA;

	template<typename T>
	void loadFields(std::istream& in, T *array);

	template<typename T>
	void saveFields(std::ostream& out, const std::string& name, T *array);

public:
	std::string version;
	std::size_t fieldCount;
	std::string *fields;
	std::size_t *size;
	char *type;
	std::size_t *count;
	std::size_t width;
	std::size_t height;
	std::string viewpoint;
	std::size_t points;
	std::string data;

	PCDHeader();
	PCDHeader(std::istream& stream);
	PCDHeader(const PCDHeader& header);
	~PCDHeader();

	void load(std::istream& stream);
	void save(std::ostream& stream);
};

template<typename T>
void PCDHeader::loadFields(std::istream& in, T *array)
{
	for(std::size_t i = 0; i < fieldCount; i++)
	{
		in >> array[i];
	}
}

template<typename T>
void PCDHeader::saveFields(std::ostream& out, const std::string& name, T *array)
{
	if(fieldCount > 0)
	{
		out << name;
		for(std::size_t i = 0; i < fieldCount; i++)
		{
			out << ' ' << array[i];
		}
		out << std::endl;
	}
}

}}

#endif // LSL_IO_PCDHEADER_HPP
