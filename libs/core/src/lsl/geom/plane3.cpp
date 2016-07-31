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

#include "lsl/geom/plane3.hpp"

namespace lsl {
namespace geom {

Plane3::Plane3(double a, double b, double c, double d) :
	a(a), b(b), c(c), d(d)
{
}

Plane3::Plane3(double a, double b, double c, double d, const std::vector<geom::Vector4d>& points) :
	a(a), b(b), c(c), d(d), points(points)
{
}

Plane3::Plane3(double a, double b, double c, double d, std::vector<geom::Vector4d>&& points) :
	a(a), b(b), c(c), d(d), points(std::move(points))
{
}

Plane3::Plane3(const geom::Vector4d& pointA, const geom::Vector4d& pointB, const geom::Vector4d& pointC, bool savePoints)
{
	geom::Vector4d vectorAB = pointB - pointA;
	geom::Vector4d vectorAC = pointC - pointA;

	geom::Vector3d normal = vectorAB.toHeterogenous().cross(vectorAC.toHeterogenous());

	a = normal[0];
	b = normal[1];
	c = normal[2];

	d = 0;
	for(std::size_t i = 0; i < 3; i++) d -= normal[i] * pointA[i];

	if(savePoints)
	{
		points.push_back(pointA);
		points.push_back(pointB);
		points.push_back(pointC);
	}
}

Plane3::Plane3(const Plane3& plane) : Plane3(plane.getA(), plane.getB(), plane.getC(), plane.getD(), plane.getPoints())
{
}

void Plane3::setParams(double a, double b, double c, double d)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
}

double Plane3::distanceTo(const geom::Vector4d& point) const
{
	return std::sqrt(distance2To(point));
}

double Plane3::distance2To(const geom::Vector4d& point) const
{
	double dt = a * point[0] + b * point[1] + c * point[2] + d;
	return dt * dt / (a * a + b * b + c * c);
}

double Plane3::sumOfDistance2To(const std::vector<geom::Vector4d>& points, double maxDistance2) const
{
	double sum = 0;
	for(const auto& point : points)
	{
		double distance2 = distance2To(point);
		if(distance2 < maxDistance2) sum += distance2;
		else sum += maxDistance2;
	}

	return sum;
}

geom::Vector3d Plane3::getNormal() const
{
	return geom::Vector3d(a, b, c);
}

geom::Vector4d Plane3::getClosestPoint(const geom::Vector4d& point) const
{
	// TODO Implement Plane3::getClosestPoint
	/*
	double cx, cy;
	if(a == 0)
	{
		cx = point[0];
		cy = -c / b;
	}
	else if(b == 0)
	{
		cx = -c / a;
		cy = point[1];
	}
	else
	{
		double one__a = 1 / a;
		cy = (a * point[1] - b * point[0] - b * c * one__a) / (b * b * one__a + a);
		cx = -(b * cy + c) * one__a;
	}

	return { cx, cy, 0, 1 };
	*/

	return {0, 0, 0, 1};
}

std::ostream& operator<<(std::ostream& out, const Plane3& plane)
{
	return out << "Plane(" << plane.a << ", " << plane.b << ", " << plane.c << ", " << plane.d << ')';
}

}}
