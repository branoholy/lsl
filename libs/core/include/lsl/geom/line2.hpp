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

#ifndef LSL_GEOM_LINE2_HPP
#define LSL_GEOM_LINE2_HPP

#include <iostream>
#include <vector>

#include "lsl/utils/cpputils.hpp"
#include "matrix.hpp"

namespace lsl {
namespace geom {

class Line2
{
private:
	double a;
	double b;
	double c;

	std::vector<geom::Vector3d> points;

public:
	Line2(double a, double b, double c);
	Line2(double a, double b, double c, const std::vector<geom::Vector3d>& points);
	Line2(double a, double b, double c, std::vector<geom::Vector3d>&& points);
	Line2(const geom::Vector3d& pointA, const geom::Vector3d& pointB, bool savePoints = false);
	Line2(const Line2& line);

	inline double getA() const { return a; }
	inline double getB() const { return b; }
	inline double getC() const { return c; }

	inline const std::vector<geom::Vector3d>& getPoints() const { return points; }

	inline double getK() const { return -a / b; }
	inline double getQ() const { return -c / b; }

	void setParams(double a, double b, double c);

	double distanceTo(const geom::Vector3d& point) const;
	double distance2To(const geom::Vector3d& point) const;
	double sumOfDistance2To(const std::vector<geom::Vector3d>& points, double maxDistance2 = std::numeric_limits<double>::max()) const;

	double getX(double y) const;
	double getY(double x) const;

	geom::Vector2d getNormal() const;

	geom::Vector3d getClosestPoint(const geom::Vector3d& point) const;

	geom::Vector3d intersect(const Line2& other) const;
	bool tryIntersect(const Line2& other, geom::Vector3d& point) const;

	std::ostream& printSlopeInterceptForm(std::ostream& out) const;

	template<typename ForwardIterator>
	static Line2 leastSquareLine(ForwardIterator begin, ForwardIterator end, bool savePoints = true);

	template<typename ForwardIterator>
	static Line2 leastSquare(ForwardIterator begin, ForwardIterator end, bool savePoints = true);

	template<typename ContainerT>
	static Line2 leastSquare(const ContainerT& points, bool savePoints = true);

	friend std::ostream& operator<<(std::ostream& out, const Line2& line);
};

template<typename ForwardIterator>
Line2 Line2::leastSquareLine(ForwardIterator begin, ForwardIterator end, bool savePoints)
{
	std::size_t size = std::distance(begin, end);
	double one__size = 1.0 / size;

	std::vector<geom::Vector3d> linePoints;
	if(savePoints) linePoints.reserve(size);

	double x_ = 0, y_ = 0;
	for(ForwardIterator it = begin; it != end; it++)
	{
		const auto *point = utils::CppUtils::getPointer(*it);

		double x = (*point)[0];
		double y = (*point)[1];

		x_ += x;
		y_ += y;

		if(savePoints) linePoints.push_back(*point);
	}
	x_ *= one__size;
	y_ *= one__size;

	double sumX_ = 0, sumY_ = 0, b_ = 0;
	for(ForwardIterator it = begin; it != end; it++)
	{
		const auto *point = utils::CppUtils::getPointer(*it);

		double x = (*point)[0];
		double y = (*point)[1];

		double x_i = x - x_;
		double y_i = y - y_;

		sumX_ += x_i * x_i;
		sumY_ += y_i * y_i;
		b_ += x_i * y_i;
	}
	double a_ = sumX_ - sumY_;

	double a = 2 * b_;
	double b = -(a_ + std::sqrt(a_ * a_ + 4 * b_ * b_));
	double c = -(a * x_ + b * y_);

	if(savePoints) return Line2(a, b, c, std::move(linePoints));
	else return Line2(a, b, c);
}

template<typename ForwardIterator>
Line2 Line2::leastSquare(ForwardIterator begin, ForwardIterator end, bool savePoints)
{
	std::size_t size = std::distance(begin, end);

	std::vector<geom::Vector3d> linePoints;
	if(savePoints) linePoints.reserve(size);

	double sumX = 0;
	double sumY = 0;
	double sumXX = 0;
	double sumYY = 0;
	double sumXY = 0;
	for(ForwardIterator it = begin; it != end; it++)
	{
		const auto *point = utils::CppUtils::getPointer(*it);

		double x = (*point)[0];
		double y = (*point)[1];

		sumX += x;
		sumY += y;
		sumXX += x * x;
		sumYY += y * y;
		sumXY += x * y;

		if(savePoints) linePoints.push_back(*point);
	}

	double a = (sumX * sumYY - sumY * sumXY) / (sumXY * sumXY - sumXX * sumYY);
	double b = (a * sumXX + sumX) / sumXY;
	double c = 1;

	if(savePoints) return Line2(a, b, c, std::move(linePoints));
	else return Line2(a, b, c);
}

template<typename ContainerT>
Line2 Line2::leastSquare(const ContainerT& points, bool savePoints)
{
	return leastSquare(points.begin(), points.end(), savePoints);
}

}}

#endif // LSL_GEOM_LINE2_HPP
