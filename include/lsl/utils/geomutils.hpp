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

#ifndef LSL_UTILS_GEOMUTILS_HPP
#define LSL_UTILS_GEOMUTILS_HPP

#include "lsl/geom/vector.hpp"

namespace lsl {
namespace utils {

class GeomUtils
{
public:
	static double dotProduct(double x, double y, double x0, double y0, double x1, double y1);
	static double distance2ToPoint(double x, double y, double x0, double y0);
	static double distance2ToLine(double x, double y, double x0, double y0, double x1, double y1);

	static geom::Vector2d getClosestLinePoint(double a, double b, double c, const geom::Vector2d& point);
	static geom::Vector2d getClosestLinePoint(double a, double b, double c, double x, double y);
};

}}

#endif // LSL_UTILS_GEOMUTILS_HPP
