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

#ifndef LSL_UTILS_ARRAYUTILS_HPP
#define LSL_UTILS_ARRAYUTILS_HPP

#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include "cpputils.hpp"

namespace lsl {
namespace utils {

class ArrayUtils
{
public:
	template <typename T>
	static T* createFilledArray(std::size_t size, T defaultValue);

	template <typename T>
	static void deleteArrayContent(T **array, std::size_t size);

	template <typename T>
	static T** create2dArray(std::size_t rowCount, std::size_t columnCount);

	template <typename T>
	static T** create2dFilledArray(std::size_t rowCount, std::size_t columnCount, T defaultValue);

	template <typename T, std::size_t rowCount, std::size_t columnCount>
	static T** create2dFilledArray(T array[rowCount][columnCount]);

	template <typename T>
	static void fill2dArray(T **array, std::size_t rowCount, std::size_t columnCount, T defaultValue);

	template <typename T>
	static void delete2dArray(T **array, std::size_t rowCount);

	template <typename T>
	static void delete2dArrayContent(T ***array, std::size_t rowCount, std::size_t columnCount);

	template <typename T>
	static void printArray(std::ostream& out, const T *array, std::size_t size);

	template <typename T>
	static void getMinMaxValues(T ** const array, std::size_t rowCount, std::size_t columnCount, T& minValue, T& maxValue, T minThreshold = std::numeric_limits<T>::min());

	static void to01(double **array, std::size_t rowCount, std::size_t columnCount, double minValue, double maxValue);

	template <typename T, typename ForwardIterator>
	static void eraseAll(std::vector<T>& v, ForwardIterator begin, ForwardIterator end);

	template <typename T>
	static void deleteAll(T **array, std::size_t size);

	template <typename T>
	static void deleteAll(const std::vector<T>& v);

	template<typename ForwardIterator>
	static void deleteAll(ForwardIterator begin, ForwardIterator end);
};

template <typename T>
T* ArrayUtils::createFilledArray(std::size_t size, T defaultValue)
{
	T *array = new T[size];
	std::fill(array, array + size, defaultValue);

	return array;
}

template <typename T>
void ArrayUtils::deleteArrayContent(T **array, std::size_t size)
{
	if(array != nullptr)
	{
		deleteAll(array, size);
		delete array;
	}
}

template <typename T>
T** ArrayUtils::create2dArray(std::size_t rowCount, std::size_t columnCount)
{
	T **array = new T*[rowCount];
	for(std::size_t r = 0; r < rowCount; r++)
	{
		array[r] = new T[columnCount];
	}

	return array;
}

template <typename T>
T** ArrayUtils::create2dFilledArray(std::size_t rowCount, std::size_t columnCount, T defaultValue)
{
	T **array = create2dArray<T>(rowCount, columnCount);
	fill2dArray(array, rowCount, columnCount, defaultValue);

	return array;
}

template <typename T, std::size_t rowCount, std::size_t columnCount>
T** ArrayUtils::create2dFilledArray(T array[rowCount][columnCount])
{
	T **output = create2dArray<T>(rowCount, columnCount);
	for(std::size_t r = 0; r < rowCount; r++)
		for(std::size_t c = 0; c < columnCount; c++)
			output[r][c] = array[r][c];

	return output;
}

template <typename T>
void ArrayUtils::fill2dArray(T **array, std::size_t rowCount, std::size_t columnCount, T defaultValue)
{
	for(std::size_t r = 0; r < rowCount; r++)
	{
		std::fill(array[r], array[r] + columnCount, defaultValue);
	}
}

template <typename T>
void ArrayUtils::delete2dArray(T **array, std::size_t rowCount)
{
	for(std::size_t r = 0; r < rowCount; r++)
	{
		delete[] array[r];
	}
	delete[] array;
}

template <typename T>
void ArrayUtils::delete2dArrayContent(T ***array, std::size_t rowCount, std::size_t columnCount)
{
	for(std::size_t r = 0; r < rowCount; r++)
	{
		for(std::size_t c = 0; c < columnCount; c++)
		{
			delete array[r][c];
		}

		delete[] array[r];
	}
	delete[] array;
}

template <typename T>
void ArrayUtils::printArray(std::ostream& out, const T* array, std::size_t size)
{
	if(size > 0)
	{
		out << array[0];
		for(std::size_t i = 1; i < size; i++)
		{
			out << /*',' <<*/ ' ' << array[i];
		}
	}
}

template <typename T>
void ArrayUtils::getMinMaxValues(T ** const array, std::size_t rowCount, std::size_t columnCount, T& minValue, T& maxValue, T minThreshold)
{
	minValue = std::numeric_limits<T>::max();
	maxValue = std::numeric_limits<T>::min();

	for(std::size_t r = 0; r < rowCount; r++)
	{
		for(std::size_t c = 0; c < columnCount; c++)
		{
			T val = array[r][c];
			if(val < minThreshold) continue;

			if(minValue > val) minValue = val;
			if(maxValue < val) maxValue = val;
		}
	}
}

template <typename T, typename ForwardIterator>
void ArrayUtils::eraseAll(std::vector<T>& v, ForwardIterator begin, ForwardIterator end)
{
	while(begin != end)
	{
		v.erase(std::remove(v.begin(), v.end(), *begin), v.end());
		begin++;
	}
}

template <typename T>
void ArrayUtils::deleteAll(T **array, std::size_t size)
{
	if(array != nullptr)
	{
		deleteAll(array, array + size);
	}
}

template <typename T>
void ArrayUtils::deleteAll(const std::vector<T>& v)
{
	deleteAll(v.begin(), v.end());
}

template<typename ForwardIterator>
void ArrayUtils::deleteAll(ForwardIterator begin, ForwardIterator end)
{
	for(ForwardIterator it = begin; it != end; it++)
	{
		delete *it;
	}
}

}}

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
{
	out << '[';
	if(v.size() > 0)
	{
		typename std::vector<T>::const_iterator it = v.begin();
		while(true)
		{
			out << *it;

			if(++it != v.end()) out << ", ";
			else break;
		}
	}

	return out << ']';
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T*>& v)
{
	out << '[';
	if(v.size() > 0)
	{
		typename std::vector<T*>::const_iterator it = v.begin();
		while(true)
		{
			if(*it == nullptr) out << "nullptr";
			else out << **it;

			if(++it != v.end()) out << ", ";
			else break;
		}
	}

	return out << ']';
}


template<typename T>
std::ostream& operator<<(std::ostream& out, const std::set<T>& v)
{
	out << '[';
	if(v.size() > 0)
	{
		typename std::set<T>::const_iterator it = v.begin();
		while(true)
		{
			out << *it;

			if(++it != v.end()) out << ", ";
			else break;
		}
	}

	return out << ']';
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::set<T*>& v)
{
	out << '[';
	if(v.size() > 0)
	{
		typename std::set<T*>::const_iterator it = v.begin();
		while(true)
		{
			if(*it == nullptr) out << "nullptr";
			else out << **it;

			if(++it != v.end()) out << ", ";
			else break;
		}
	}

	return out << ']';
}

template<typename Key, typename T>
std::ostream& operator<<(std::ostream& out, const std::map<Key, T>& m)
{
	out << '{';
	if(m.size() > 0)
	{
		typename std::map<Key, T>::const_iterator it = m.begin();
		while(true)
		{
			out << it->first << ": " << it->second;

			if(++it != m.end()) out << ", ";
			else break;
		}
	}

	return out << '}';
}

template<typename Key, typename T>
std::ostream& operator<<(std::ostream& out, const std::map<Key, T*>& m)
{
	out << '{';
	if(m.size() > 0)
	{
		typename std::map<Key, T*>::const_iterator it = m.begin();
		while(true)
		{
			out << it->first << ": ";

			if(*it->second == nullptr) out << "nullptr";
			else out << *it->second;

			if(++it != m.end()) out << ", ";
			else break;
		}
	}

	return out << '}';
}

#endif // LSL_UTILS_ARRAYUTILS_HPP
