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

#ifndef LSL_UTILS_STRINGUTILS_HPP
#define LSL_UTILS_STRINGUTILS_HPP

#include <sstream>
#include <iomanip>

namespace lsl {
namespace utils {

class StringUtils
{
public:
	template <typename T>
	static std::string toString(const T& value, int precision = 6, bool fixed = true);
};

template <typename T>
std::string StringUtils::toString(const T& value, int precision, bool fixed)
{
	std::ostringstream out;
	if(fixed) out.setf(std::ios_base::fixed);

	out << std::setprecision(precision) << value;
	return out.str();
}

}}

#endif // LSL_UTILS_STRINGUTILS_HPP
