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

#include "lsl/utils/mathutils.hpp"

#include <cmath>
#include <limits>

namespace lsl {
namespace utils {

const double MathUtils::PI = M_PI;
const double MathUtils::TWO_PI = 2 * MathUtils::PI;
const double MathUtils::TWO_PI_EXCLUSIVE = MathUtils::TWO_PI - 10 * std::numeric_limits<double>::epsilon();
const double MathUtils::PI__TWO = MathUtils::PI / 2;
const double MathUtils::PI__FOUR = MathUtils::PI / 4;
const double MathUtils::THREE_PI__TWO = 3 * MathUtils::PI__TWO;
const double MathUtils::ONE__TWO_PI = 1 / (2 * MathUtils::PI);
const double MathUtils::SQRT_TWO_PI = std::sqrt(MathUtils::TWO_PI);

const double MathUtils::EPSILON_10 = 0.0000000001;

double MathUtils::normAngle(double angle)
{
	while(angle < 0 || angle >= TWO_PI)
	{
		angle = std::fmod(angle + TWO_PI, TWO_PI);
	}

	return angle;
}

double MathUtils::normAnglePi(double angle)
{
	while(angle < -PI) angle += TWO_PI;
	while(angle >= PI) angle -= TWO_PI;

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

double cot(double x)
{
	return 1 / std::tan(x);
}

double sec(double x)
{
	return 1 / std::cos(x);
}

double sec2(double x)
{
	double c = std::cos(x);
	return 1 / (c * c);
}

double csc(double x)
{
	return 1 / std::sin(x);
}

double csc2(double x)
{
	double s = std::sin(x);
	return 1 / (s * s);
}

double intCos2(double x, double a)
{
	double x_ = 2 * (x + a);
	return 0.25 * (x_ + std::sin(x_));
}

double intCos2(double x, double a, double b)
{
	double x_ = 2 * x;
	return 0.25 * (x_ * std::cos(a - b) + std::sin(a + b + x_));
}

}}
