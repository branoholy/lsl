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

#include <cstdio>
#include <iomanip>
#include <sstream>

namespace lsl {
namespace utils {

template <typename CharT>
class BasicStringUtils
{
public:
	template <typename T>
	static std::basic_string<CharT> toString(const T& value, int precision = 6, bool fixed = true);

	template <typename ...Args>
	static std::basic_string<CharT> format(const std::basic_string<CharT>& format, Args... args);
};

template <typename CharT>
template <typename T>
std::basic_string<CharT> BasicStringUtils<CharT>::toString(const T& value, int precision, bool fixed)
{
	std::basic_ostringstream<CharT> out;
	if(fixed) out.setf(std::ios_base::fixed);

	out << std::setprecision(precision) << value;
	return out.str();
}

template <typename CharT>
template <typename ...Args>
std::basic_string<CharT> BasicStringUtils<CharT>::format(const std::basic_string<CharT>& format, Args... args)
{
	typename std::basic_string<CharT>::size_type size = std::snprintf(nullptr, 0, format.c_str(), args...);

	std::basic_string<CharT> formatted;
	formatted.reserve(size + 1);
	formatted.resize(size);
	std::sprintf(&formatted[0], size + 1, format.c_str(), args...);

	return formatted;
}

typedef BasicStringUtils<char> StringUtils;
typedef BasicStringUtils<wchar_t> WStringUtils;

}}

#endif // LSL_UTILS_STRINGUTILS_HPP
