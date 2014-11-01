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

#include "lsl/geom/lidarline2.hpp"

#include <cmath>
#include <limits>

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace geom {

LidarLine2::LidarLine2(const Line2& line)
{
	v = line.getNormal();
	double norm = -line.getC() / v.getLength2();
	v *= norm;
}

LidarLine2::LidarLine2(const Line2& line, const Vector2d& endPointA, const Vector2d& endPointB) : LidarLine2(line)
{
	setEndPointA(endPointA);
	setEndPointB(endPointB);
}

double LidarLine2::getValue(double alpha) const
{
	return getL() / cos(alpha - getPhi());
}

void LidarLine2::setPhiA(double phiA)
{
	this->phiA = phiA;

	double value = getValue(phiA);
	endPointA.set(0, value * cos(phiA));
	endPointA.set(1, value * sin(phiA));
}

void LidarLine2::setPhiB(double phiB)
{
	this->phiB = phiB;

	double value = getValue(phiB);
	endPointB.set(0, value * cos(phiB));
	endPointB.set(1, value * sin(phiB));
}

void LidarLine2::setEndPointA(const Vector2d& endPointA)
{
	this->endPointA.set(endPointA);
	phiA = endPointA.getAngle2D();
}

void LidarLine2::setEndPointB(const Vector2d& endPointB)
{
	this->endPointB.set(endPointB);
	phiB = endPointB.getAngle2D();
}

void LidarLine2::transform(double angle, double tx, double ty)
{
	double c = cos(angle);
	double s = sin(angle);

	transform(c, s, tx, ty);
}

void LidarLine2::transform(double c, double s, double tx, double ty)
{
	v.rotate2D(c, s);

	double l2 = v.getLength2();
	double vx = v.get(0);
	double vy = v.get(1);

	double vx_ = vx + (vx * vx * tx + vx * vy * ty) / l2;
	double vy_ = vy + (vx * vy * tx + vy * vy * ty) / l2;

	v.set(0, vx_);
	v.set(1, vy_);

	endPointA.transform2D(c, s, tx, ty);
	endPointB.transform2D(c, s, tx, ty);

	phiA = endPointA.getAngle2D();
	phiB = endPointB.getAngle2D();
}

double LidarLine2::error(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double l0 = getL();
	double l02 = l0 * l0;
	double phi0 = getPhi();

	double l1 = other.getL();
	double l12 = l1 * l1;
	double phi1 = other.getPhi();

	double one__sin = 2 * l0 * l1;
	double errLow = 0;
	double errHigh = 0;

	if(abs(phi0 - phi1) < numeric_limits<double>::epsilon())
	{
		double tanLow = tan(phiLow - phi0);
		double tanHigh = tan(phiHigh - phi0);

		errLow = l02 * tanLow + l12 * tanLow - one__sin * tanLow;
		errHigh = l02 * tanHigh + l12 * tanHigh - one__sin * tanHigh;
	}
	else
	{
		one__sin /= sin(phi0 - phi1);
		errLow = l02 * tan(phiLow - phi0) + l12 * tan(phiLow - phi1) - one__sin * (log(cos(phi0 - phiLow)) - log(cos(phi1 - phiLow)));
		errHigh = l02 * tan(phiHigh - phi0) + l12 * tan(phiHigh - phi1) - one__sin * (log(cos(phi0 - phiHigh)) - log(cos(phi1 - phiHigh)));
	}

	return errHigh - errLow;
}

ostream& operator<<(ostream& out, const LidarLine2& lidarLine)
{
	return out << "LL(" << lidarLine.v << " |" << lidarLine.getL() << "|, " << lidarLine.getPhi() << ')';
}

}}
