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

#include "lsl/geom/line2.hpp"

#include "lsl/utils/geomutils.hpp"

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace geom {

Line2::Line2(double a, double b, double c) :
	a(a), b(b), c(c)
{
}

Line2::Line2(double a, double b, double c, const vector<Vector2d>& points) :
	a(a), b(b), c(c), points(points)
{
}

Line2::Line2(double a, double b, double c, vector<Vector2d>&& points) :
	a(a), b(b), c(c), points(std::move(points))
{
}

Line2::Line2(const Line2& line) : Line2(line.getA(), line.getB(), line.getC())
{
}

void Line2::setParams(double a, double b, double c)
{
	this->a = a;
	this->b = b;
	this->c = c;
}

double Line2::distanceTo(const Vector2d& point) const
{
	return abs(a * point[0] + b * point[1] + c) / sqrt(a * a + b * b);
}

double Line2::distance2To(const Vector2d& point) const
{
	double dt = a * point[0] + b * point[1] + c;
	return dt * dt / (a * a + b * b);
}

double Line2::getX(double y)
{
	return -(b * y + c) / a;
}

double Line2::getY(double x)
{
	return -(a * x + c) / b;
}

Vector2d Line2::getNormal() const
{
	return Vector2d({a, b});
}

Vector2d Line2::getOrientedNormal() const
{
	Vector2d n = getNormal();

	if(points.size() > 1)
	{
		Vector2d pa = points.at(0);
		Vector2d pb = points.at(1);

		Vector2d n_ {pa[1] - pb[1], pb[0] - pa[0]};
		n_ *= pb.getId() - pa.getId();

		// abs(n * n_) ???
		double cosAlpha = (n * n_) / (n.getLength() * n_.getLength());
		if(cosAlpha != 0)
		{
			n *= cosAlpha;
		}
	}

	return n;
}

Vector2d Line2::getClosestPoint(const Vector2d& point) const
{
	return GeomUtils::getClosestLinePoint(a, b, c, point);
}

ostream& Line2::printSlopeInterceptForm(ostream& out) const
{
	return out << "Line-SI(y = " << getK() << "*x " << showpos << getQ() << noshowpos << ')';
}

ostream& operator<<(ostream& out, const Line2& line)
{
	return out << "Line(" << line.a << ", " << line.b << ", " << line.c << ')';
}

}}
