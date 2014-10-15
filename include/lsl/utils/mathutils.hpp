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

#ifndef LSL_UTILS_MATHUTILS_HPP
#define LSL_UTILS_MATHUTILS_HPP

// #include <cstring>

namespace lsl
{
	namespace utils
	{
		class MathUtils
		{
		public:
			static const double PI;
			static const double TWO_PI;
			static const double ONE__TWO_PI;

			// static double isNegative(double x, double c = 10000000000000.0);
			static double normAngle(double angle);

			// static size_t cyclicDistance(size_t i, size_t j, size_t size);
			// static size_t cyclicDistance(size_t i, size_t j, size_t size, bool& isCyclic);
			// static bool isCyclic(size_t i, size_t j, size_t size);
			// static size_t cyclicIncrement(size_t i, int direction, size_t size);
		};
	}
}

#endif // LSL_UTILS_MATHUTILS_HPP
