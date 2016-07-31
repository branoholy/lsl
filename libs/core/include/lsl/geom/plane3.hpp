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

#ifndef LSL_GEOM_PLANE3_HPP
#define LSL_GEOM_PLANE3_HPP

#include <iostream>
#include <vector>

#include "lsl/utils/cpputils.hpp"
#include "matrix.hpp"

namespace lsl {
namespace geom {

class Plane3
{
private:
	double a;
	double b;
	double c;
	double d;

	std::vector<geom::Vector4d> points;

public:
	Plane3(double a, double b, double c, double d);
	Plane3(double a, double b, double c, double d, const std::vector<geom::Vector4d>& points);
	Plane3(double a, double b, double c, double d, std::vector<geom::Vector4d>&& points);
	Plane3(const geom::Vector4d& pointA, const geom::Vector4d& pointB, const geom::Vector4d& pointC, bool savePoints = false);
	Plane3(const Plane3& plane);

	inline double getA() const { return a; }
	inline double getB() const { return b; }
	inline double getC() const { return c; }
	inline double getD() const { return d; }

	inline const std::vector<geom::Vector4d>& getPoints() const { return points; }

	void setParams(double a, double b, double c, double d);

	double distanceTo(const geom::Vector4d& point) const;
	double distance2To(const geom::Vector4d& point) const;
	double sumOfDistance2To(const std::vector<geom::Vector4d>& points, double maxDistance2 = std::numeric_limits<double>::max()) const;

	geom::Vector3d getNormal() const;

	geom::Vector4d getClosestPoint(const geom::Vector4d& point) const;

	template<typename ForwardIterator>
	static Plane3 leastSquare(ForwardIterator begin, ForwardIterator end, bool savePoints = true);

	template<typename ContainerT>
	static Plane3 leastSquare(const ContainerT& points, bool savePoints = true);

	friend std::ostream& operator<<(std::ostream& out, const Plane3& plane);
};

template<typename ForwardIterator>
Plane3 Plane3::leastSquare(ForwardIterator begin, ForwardIterator end, bool savePoints)
{
	// TODO Implement Plane3::leastSquare
	/*
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
	double d = 21356;

	if(savePoints) return Plane3(a, b, c, d, std::move(linePoints));
	else return Plane3(a, b, c, d);
	*/

	return Plane3(0, 0, 0, 0);
}

template<typename ContainerT>
Plane3 Plane3::leastSquare(const ContainerT& points, bool savePoints)
{
	return leastSquare(points.begin(), points.end(), savePoints);
}

}}

#endif // LSL_GEOM_PLANE3_HPP
