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

#ifndef LSL_UTILS_CPPUTILS_HPP
#define LSL_UTILS_CPPUTILS_HPP

#include <type_traits>

namespace lsl {
namespace utils {

class CppUtils
{
public:
	template<typename T>
	static T* getPointer(T& object);

	template<typename T>
	static const T* getPointer(const T& object);

	template<typename T>
	static T* getPointer(T *object);

	template<typename T>
	static const T* getPointer(const T *object);

	template<typename R, typename T>
	static typename std::enable_if<std::is_pointer<R>::value, R>::type getValue(T object);

	template<typename R, typename T>
	static R getValue(T object);
};

template<typename T>
T* CppUtils::getPointer(T& object)
{
	return &object;
}

template<typename T>
const T* CppUtils::getPointer(const T& object)
{
	return &object;
}

template<typename T>
T* CppUtils::getPointer(T *object)
{
	return object;
}

template<typename T>
const T* CppUtils::getPointer(const T *object)
{
	return object;
}

template<typename R, typename T>
typename std::enable_if<std::is_pointer<R>::value, R>::type CppUtils::getValue(T object)
{
	return object;
}

template<typename R, typename T>
R CppUtils::getValue(T object)
{
	return *object;
}

}}

#endif // LSL_UTILS_CPPUTILS_HPP
