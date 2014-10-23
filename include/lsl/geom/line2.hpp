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

#ifndef LSL_GEOM_LINE2_HPP
#define LSL_GEOM_LINE2_HPP

#include <vector>
#include <iostream>

#include "vector.hpp"
#include "lsl/utils/cpputils.hpp"

namespace lsl {
namespace geom {

class Line2
{
private:
	double a;
	double b;
	double c;

	std::vector<Vector2d> points;
public:
	Line2(double a, double b, double c);
	Line2(double a, double b, double c, std::vector<Vector2d> points);
	Line2(const Line2& line);

	inline double getA() const { return a; }
	inline double getB() const { return b; }
	inline double getC() const { return c; }

	inline double getK() const { return - a / b; }
	inline double getQ() const { return - c / b; }

	void setParams(double a, double b, double c);

	double distanceTo(const Vector2d& point) const;
	double distance2To(const Vector2d& point) const;

	double getX(double y);
	double getY(double x);

	Vector2d getNormal() const;
	Vector2d getOrientedNormal() const;

	Vector2d getClosestPoint(const Vector2d& point) const;

	std::ostream& printSlopeInterceptForm(std::ostream& out) const;

	template <typename ContainerT>
	static Line2 leastSquareLine(ContainerT points);

	friend std::ostream& operator<<(std::ostream& out, const Line2& line);
};

template <typename ContainerT>
Line2 Line2::leastSquareLine(ContainerT points)
{
	typedef typename std::remove_pointer<typename ContainerT::value_type>::type PointType;

	double sumX = 0;
	double sumXX = 0;
	double sumY = 0;
	double sumXY = 0;
	int size = points.size();
	std::vector<Vector2d> linePoints;

	for(typename ContainerT::iterator it = points.begin(); it != points.end(); it++)
	{
		typename ContainerT::value_type pointOrig = *it;
		PointType *point = utils::CppUtils::getPointer(pointOrig);

		sumX += point->get(0);
		sumXX += point->get(0) * point->get(0);
		sumY += point->get(1);
		sumXY += point->get(0) * point->get(1);
		linePoints.push_back(*point);
	}

	double d = size * sumXX - sumX * sumX;

	double a = 0;
	double b = 0;
	double c = 0;

	if(d == 0)
	{
		a = 1;
		b = 0;
		c = -sumX / size;
	}
	else
	{
		double k = (size * sumXY - sumX * sumY) / d;
		double q = (sumY * sumXX - sumX * sumXY) / d;

		a = k;
		b = -1;
		c = q;
	}

	return Line2(a, b, c, linePoints);
}

}}

#endif // LSL_GEOM_LINE2_HPP
