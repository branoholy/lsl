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

#include "lsl/geom/line2.hpp"

namespace lsl {
namespace geom {

Line2::Line2(double a, double b, double c) :
	a(a), b(b), c(c)
{
}

Line2::Line2(double a, double b, double c, const std::vector<geom::Vector3d>& points) :
	a(a), b(b), c(c), points(points)
{
}

Line2::Line2(double a, double b, double c, std::vector<geom::Vector3d>&& points) :
	a(a), b(b), c(c), points(std::move(points))
{
}

Line2::Line2(const geom::Vector3d& pointA, const geom::Vector3d& pointB, bool savePoints)
{
	geom::Vector3d normal = pointB - pointA;
	std::swap(normal[0], normal[1]);
	normal[0] *= -1;

	a = normal[0];
	b = normal[1];

	c = 0;
	for(std::size_t i = 0; i < 2; i++)
		c -= normal[i] * pointA[i];

	if(savePoints)
	{
		points.push_back(pointA);
		points.push_back(pointB);
	}
}

Line2::Line2(const Line2& line) : Line2(line.getA(), line.getB(), line.getC(), line.getPoints())
{
}

void Line2::setParams(double a, double b, double c)
{
	this->a = a;
	this->b = b;
	this->c = c;
}

double Line2::distanceTo(const geom::Vector3d& point) const
{
	return std::sqrt(distance2To(point));
}

double Line2::distance2To(const geom::Vector3d& point) const
{
	double dt = a * point[0] + b * point[1] + c;
	return dt * dt / (a * a + b * b);
}

double Line2::sumOfDistance2To(const std::vector<geom::Vector3d>& points, double maxDistance2) const
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

double Line2::getX(double y) const
{
	return -(b * y + c) / a;
}

double Line2::getY(double x) const
{
	return -(a * x + c) / b;
}

geom::Vector2d Line2::getNormal() const
{
	return geom::Vector2d(a, b);
}

geom::Vector3d Line2::getClosestPoint(const geom::Vector3d& point) const
{
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

	return { cx, cy, 1 };
}

geom::Vector3d Line2::intersect(const Line2& other) const
{
	geom::Vector3d point;
	if(!tryIntersect(other, point))
		throw std::invalid_argument("Lines are parallel.");

	return point;
}

bool Line2::tryIntersect(const Line2& other, geom::Vector3d& point) const
{
	double determinant = b * other.a - a * other.b;
	if(determinant == 0) return false;

	point[0] = (c * other.b - b * other.c) / determinant;
	point[1] = (a * other.c - c * other.a) / determinant;

	return true;
}

std::ostream& Line2::printSlopeInterceptForm(std::ostream& out) const
{
	return out << "Line-SI(y = " << getK() << "*x " << std::showpos << getQ() << std::noshowpos << ')';
}

std::ostream& operator<<(std::ostream& out, const Line2& line)
{
	return out << "Line(" << line.a << ", " << line.b << ", " << line.c << ')';
}

}}
