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

#include "lsl/utils/geomutils.hpp"

#include <cmath>

using namespace std;
using namespace lsl::geom;

namespace lsl {
namespace utils {

double GeomUtils::dotProduct(double x, double y, double x0, double y0, double x1, double y1)
{
	return (x0 - x1) * (x0 - x) + (y0 - y1) * (y0 - y);
}

double GeomUtils::distance2ToPoint(double x, double y, double x0, double y0)
{
	return pow(x0 - x, 2) + pow(y0 - y, 2);
}

double GeomUtils::distance2ToLine(double x, double y, double x0, double y0, double x1, double y1)
{
	return pow((y1 - y0) * x - (x1 - x0) * y - x0 * y1 + x1 * y0, 2) / distance2ToPoint(x1, y1, x0, y0);
}

Vector2d GeomUtils::getClosestLinePoint(double a, double b, double c, const Vector2d& point)
{
	return getClosestLinePoint(a, b, c, point[0], point[1]);
}

Vector2d GeomUtils::getClosestLinePoint(double a, double b, double c, double x, double y)
{
	double cx, cy;

	if(a == 0)
	{
		cx = x;
		cy = -c / b;
	}
	else if(b == 0)
	{
		cx = -c / a;
		cy = y;
	}
	else
	{
		cy = (a * y - b * x - (b * c) / a) / (a + (b * b) / a);
		cx = (-b * cy - c) / a;
	}

	return Vector2d({cx, cy});
}

}}
