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
#include <limits>
// #include <algorithm>

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace utils {

const double MathUtils::PI = M_PI;
const double MathUtils::TWO_PI = 2 * PI;
const double MathUtils::TWO_PI_EXCLUSIVE = TWO_PI - 10 * numeric_limits<double>::epsilon();
const double MathUtils::PI__TWO = PI / 2;
const double MathUtils::THREE_PI__TWO = 3 * PI__TWO;
const double MathUtils::ONE__TWO_PI = 1 / (2 * PI);
const double MathUtils::SQRT_TWO_PI = sqrt(TWO_PI);

double MathUtils::normAngle(double angle)
{
	while(angle < 0 || angle >= TWO_PI)
	{
		angle = fmod(angle + TWO_PI, TWO_PI);
	}

	return angle;
}

double MathUtils::toDegrees(double radians)
{
	return 180 * radians / PI;
}

double MathUtils::toRadians(double degrees)
{
	return PI * degrees / 180;
}

double MathUtils::to01(double value, double minValue, double maxValue)
{
	if(value < minValue) value = minValue;
	else if(value > maxValue) value = maxValue;

	return (value - minValue) / (maxValue - minValue);
}

}}
