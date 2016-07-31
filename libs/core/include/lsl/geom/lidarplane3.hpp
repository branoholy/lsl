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

#ifndef LSL_GEOM_LIDARPLANE3_HPP
#define LSL_GEOM_LIDARPLANE3_HPP

#include <iostream>

#include "plane3.hpp"
#include "transformable.hpp"

namespace lsl {
namespace geom {

class LidarPlane3 : public Transformable<double, 3>
{
private:
	double l; ///< A distance from the origin.
	geom::Vector2d normal; ///< Angles of the normal vector.

	geom::Vector2d beginBound; ///< Low angle bounds.
	geom::Vector2d endBound; ///< High angle bounds.

	geom::Vector4d beginBoundPoint; ///< A low bound of the bounding box.
	geom::Vector4d endBoundPoint; ///< A high bound of the bounding box.

	geom::Vector2d computeNormalAngles(const Plane3& plane) const;
	void computeNormalAngles(const Plane3& plane, geom::Vector2d& normalAngles) const;
	geom::Vector2d computeBound(const geom::Vector4d& boundPoint) const;

	void set(const Plane3& plane, const geom::Vector4d& beginBoundPoint, const geom::Vector4d& endBoundPoint);

	double getRawDistance(const geom::Vector2d& angles) const;
	double getRawCloseness(const geom::Vector2d& angles) const;
	geom::Vector4d getRawPoint(const geom::Vector2d& angles) const;

	void setBounds(const geom::Vector2d& beginBound, const geom::Vector2d& endBound, bool check = true);
	void checkBounds(const geom::Vector2d& normal, const geom::Vector2d& beginBound, const geom::Vector2d& endBound);

public:
	int id = -1;

	LidarPlane3(double l, const geom::Vector2d& normal, const geom::Vector2d& beginBound, const geom::Vector2d& endBound, int id = -1);
	LidarPlane3(const Plane3& plane, const geom::Vector4d& beginBoundPoint, const geom::Vector4d& endBoundPoint, int id = -1);
	LidarPlane3(const std::vector<geom::Vector4d>& points, int id = -1);

	inline double getL() const { return l; }
	inline const geom::Vector2d& getBeginBound() const { return beginBound; }
	inline const geom::Vector2d& getEndBound() const { return endBound; }

	double getDistance(const geom::Vector2d& angles) const;
	double getPlaneDistance(const geom::Vector2d& angles) const;

	double getCloseness(const geom::Vector2d& angles) const;
	double getPlaneCloseness(const geom::Vector2d& angles) const;

	geom::Vector4d getPoint(const geom::Vector2d& angles) const;
	geom::Vector4d getPlanePoint(const geom::Vector2d& angles) const;

	double getLowBound(std::size_t index) const;
	geom::Vector2d getLowBound() const;

	double getHighBound(std::size_t index) const;
	geom::Vector2d getHighBound() const;

	double getLowDomain(std::size_t index) const;
	geom::Vector2d getLowDomain() const;

	double getHighDomain(std::size_t index) const;
	geom::Vector2d getHighDomain() const;

	bool isVisible() const;
	bool inBounds(const geom::Vector2d& angles) const;
	bool inDomain(const geom::Vector2d& angles) const;

	void transform(const Transformation& transformation);

	static void transformAll(std::vector<LidarPlane3>& lidarPlanes, const Transformation& transformation, bool removeInvalid = false);
	static void transformAllToLocation(std::vector<LidarPlane3>& lidarPlanes, const Location& location, bool removeInvalid = false);

	double error(const LidarPlane3& other, const geom::Vector2d& lowBound, const geom::Vector2d& highBound) const;
	Location gradientErrorAtZero(const LidarPlane3& other, const geom::Vector2d& lowBound, const geom::Vector2d& highBound) const;

	bool operator==(const LidarPlane3& other) const;

	friend std::ostream& operator<<(std::ostream& out, const LidarPlane3& lidarPlane);
};

}}

#endif // LSL_GEOM_LIDARPLANE3_HPP
