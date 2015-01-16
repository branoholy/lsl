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

#ifndef LSL_UTILS_COLORUTILS_HPP
#define LSL_UTILS_COLORUTILS_HPP

#include <cmath>

namespace lsl {
namespace utils {

class ColorUtils
{
public:
	static double DEFAULT_MAX_H;

	static void hsv2rgb(double h, double s, double v, double& r, double& g, double& b);
	static void hsv2rgb(double h, double s, double v, int& r, int& g, int& b);
	static unsigned long hsv2bgr(double h, double s, double v);

	static void range2rgb(double value, double& r, double& g, double& b, double maxH = DEFAULT_MAX_H);
	static void range2rgb(double value, int& r, int& g, int& b, double maxH = DEFAULT_MAX_H);
	static unsigned long range2bgr(double value, double maxH = DEFAULT_MAX_H);
};

}}

#endif // LSL_UTILS_COLORUTILS_HPP
