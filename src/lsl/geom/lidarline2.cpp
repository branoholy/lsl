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

#include "lsl/utils/mathutils.hpp"

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

double LidarLine2::getL() const
{
	return v.getLength();
}

double LidarLine2::getPhi() const
{
	return MathUtils::normAngle(atan2(v.get(1), v.get(0)));
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

	double vx_ = vx + vx * vx * tx / l2 + vx * vy * ty / l2;
	double vy_ = vy + vx * vy * tx / l2 + vy * vy * ty / l2;

	v.set(0, vx_);
	v.set(1, vy_);
}

double LidarLine2::error(const LidarLine2& other, double phiA, double phiB)
{
	double l0 = getL();
	double l02 = l0 * l0;
	double phi0 = getPhi();

	double l1 = other.getL();
	double l12 = l1 * l1;
	double phi1 = other.getPhi();

	// if phi0 == phi1 .. abs(phi0 - phi1) < epsilon, use cos^2 formula
	double one__sin = 2 * l0 * l1;
	double errA = 0;
	double errB = 0;

	if(abs(phi0 - phi1) < numeric_limits<double>::epsilon())
	{
		double tanA = tan(phiA - phi0);
		double tanB = tan(phiB - phi0);

		errA = l02 * tanA + l12 * tanA - one__sin * tanA;
		errB = l02 * tanB + l12 * tanB - one__sin * tanB;
	}
	else
	{
		one__sin *= 1.0 / (sin(phi0 - phi1));
		errA = l02 * tan(phiA - phi0) + l12 * tan(phiA - phi1) - one__sin * (log(cos(phi0 - phiA)) - log(cos(phi1 - phiA)));
		errB = l02 * tan(phiB - phi0) + l12 * tan(phiB - phi1) - one__sin * (log(cos(phi0 - phiB)) - log(cos(phi1 - phiB)));
	}

	return errB - errA;
}

ostream& operator<<(ostream& out, const LidarLine2& lidarLine)
{
	return out << "LL(" << lidarLine.v << " |" << lidarLine.getL() << "|, " << lidarLine.getPhi() << ')';
}

}}
