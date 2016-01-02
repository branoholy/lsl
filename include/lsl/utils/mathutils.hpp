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

#ifndef LSL_UTILS_MATHUTILS_HPP
#define LSL_UTILS_MATHUTILS_HPP

namespace lsl {
namespace utils {

class MathUtils
{
public:
	static const double PI;
	static const double TWO_PI;
	static const double TWO_PI_EXCLUSIVE;
	static const double PI__TWO;
	static const double PI__FOUR;
	static const double THREE_PI__TWO;
	static const double ONE__TWO_PI;
	static const double SQRT_TWO_PI;

	static const double EPSILON_10;

	static double normAngle(double angle);
	static double normAnglePi(double angle);

	static double toDegrees(double radians);
	static double toRadians(double degrees);

	static double to01(double value, double minValue, double maxValue);
};

double cot(double x);
double sec(double x);
double sec2(double x);
double csc(double x);
double csc2(double x);

}}

#endif // LSL_UTILS_MATHUTILS_HPP
