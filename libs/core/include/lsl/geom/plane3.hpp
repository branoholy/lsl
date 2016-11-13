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

	std::size_t size = std::distance(begin, end);
	std::vector<geom::Vector4d> planePoints;
	if(savePoints)
	{
		planePoints.reserve(size);
		planePoints.insert(planePoints.begin(), begin, end);
	}

	geom::Vector4d A = *begin;
	geom::Vector4d AB = (*(begin + 1) - A);
	geom::Vector4d AC = (*(begin + 2) - A);
	geom::Vector3d n = AB.toHeterogenous().cross(AC.toHeterogenous());

	double a = n[0];
	double b = n[1];
	double c = n[2];
	double d = -(a * A[0] + b * A[1] + c * A[2]);

	if(savePoints) return Plane3(a, b, c, d, std::move(planePoints));
	else return Plane3(a, b, c, d);
}

template<typename ContainerT>
Plane3 Plane3::leastSquare(const ContainerT& points, bool savePoints)
{
	return leastSquare(points.begin(), points.end(), savePoints);
}

}}

#endif // LSL_GEOM_PLANE3_HPP
