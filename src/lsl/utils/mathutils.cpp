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

#include "lsl/utils/mathutils.hpp"

#include <cmath>
// #include <algorithm>

using namespace std;
using namespace lsl::utils;

namespace lsl
{
	namespace utils
	{
		const double MathUtils::PI = M_PI;
		const double MathUtils::TWO_PI = 2 * M_PI;
		const double MathUtils::ONE__TWO_PI = 1 / (2 * PI);
	}
}

double MathUtils::normAngle(double angle)
{
	while(angle < 0 || angle >= TWO_PI)
	{
		angle = fmod(angle + TWO_PI, TWO_PI);
	}

	return angle;
}
