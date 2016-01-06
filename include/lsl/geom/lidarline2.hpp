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

#ifndef LSL_GEOM_LIDARLINE2_HPP
#define LSL_GEOM_LIDARLINE2_HPP

#include <iostream>

#include "line2.hpp"
#include "transformable.hpp"

namespace lsl {
namespace geom {

class LidarLine2 : public Transformable<double, 2>
{
private:
	double l;
	double alpha;

	double phiA;
	double phiB;

	geom::Vector3d endPointA;
	geom::Vector3d endPointB;

	bool managed;

	void setLine(const Line2& line);

	void setEndPointA(double phiA);
	void setEndPointA(const geom::Vector3d& endPointA, bool testBounds_);

	void setEndPointB(double phiB);
	void setEndPointB(const geom::Vector3d& endPointB, bool testBounds_);

	void testBounds(double alpha, double phiA, double phiB);
	void testBounds(const Line2& line, const geom::Vector3d& endPointA, const geom::Vector3d& endPointB);

public:
	LidarLine2(double l, double alpha, double phiA, double phiB, bool managed = true);
	LidarLine2(const Line2& line, const geom::Vector3d& endPointA, const geom::Vector3d& endPointB, bool managed = true);
	LidarLine2(const geom::Vector3d& endPointA, const geom::Vector3d& endPointB, bool managed = true);
	LidarLine2(const std::vector<geom::Vector3d>& points, bool managed = true);

	std::size_t *intervalEndIndexA;
	std::size_t *intervalEndIndexB;

	inline double getL() const { return l; }
	inline double getAlpha() const { return alpha; }

	inline bool getManaged() const { return managed; }
	inline void setManaged(bool managed) { this->managed = managed; }

	void set(const Line2& line);
	void set(const Line2& line, const geom::Vector3d& endPointA, const geom::Vector3d& endPointB);

	double getValue(double phi) const;
	double getLineValue(double phi) const;

	inline double getPhiA() const { return phiA; }
	void setPhiA(double phiA);

	inline double getPhiB() const { return phiB; }
	void setPhiB(double phiB);

	void setPhiAB(double phiA, double phiB);

	inline geom::Vector3d getEndPointA() const { return endPointA; }
	void setEndPointA(const geom::Vector3d& endPointA);

	inline geom::Vector3d getEndPointB() const { return endPointB; }
	void setEndPointB(const geom::Vector3d& endPointB);

	double getPhiLow() const;
	double getPhiHigh() const;

	double getDomainLow() const;
	double getDomainHigh() const;

	bool isVisible() const;
	bool inBounds(double phi) const;
	bool inDomain(double phi) const;
	bool checkBounds() const;

	void transform(const Transformation& transformation);

	static void transformAll(std::vector<LidarLine2>& lidarLines, const Transformation& transformation, bool removeInvalid = false);
	static void transformAllToLocation(std::vector<LidarLine2>& lidarLines, const Location& location, bool removeInvalid = false);

	double error(const LidarLine2& other, double phiLow, double phiHigh) const;
	geom::Vector3d gradientErrorAtZero(const LidarLine2& other, double phiLow, double phiHigh) const;

	bool operator==(const LidarLine2& other) const;

	static double sumDf(const std::vector<LidarLine2>& lidarLines);

	friend std::ostream& operator<<(std::ostream& out, const LidarLine2& lidarLine);
};

}}

#endif // LSL_GEOM_LIDARLINE2_HPP
