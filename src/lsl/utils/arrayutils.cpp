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

#include "lsl/utils/arrayutils.hpp"

using namespace std;

namespace lsl {
namespace utils {

void ArrayUtils::to01(double **array, std::size_t rowCount, std::size_t columnCount, double minValue, double maxValue)
{
	double one__range = 1.0 / (maxValue - minValue);

	for(size_t r = 0; r < rowCount; r++)
	{
		for(size_t c = 0; c < columnCount; c++)
		{
			double value = array[r][c];
			if(value < minValue) value = minValue;
			if(value > maxValue) value = maxValue;

			if(value != 0) array[r][c] = (value - minValue) * one__range;
		}
	}
}

}}
