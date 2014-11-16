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
#include <iostream>

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace geom {

LidarLine2::LidarLine2(const Line2& line)
{
	Vector2d normal = line.getNormal();
	l = abs(line.getC() / normal.getLength());

	normal *= -line.getC();
	alpha = normal.getAngle2D();
}

LidarLine2::LidarLine2(const Line2& line, const Vector2d& endPointA, const Vector2d& endPointB) : LidarLine2(line)
{
	setEndPointA(endPointA);
	setEndPointB(endPointB);
}

double LidarLine2::getValue(double phi) const
{
	return l / cos(phi - alpha);
}

void LidarLine2::setPhiA(double phiA)
{
	this->phiA = phiA;

	double value = getValue(phiA);
	endPointA[0] = value * cos(phiA);
	endPointA[1] = value * sin(phiA);
}

void LidarLine2::setPhiB(double phiB)
{
	this->phiB = phiB;

	double value = getValue(phiB);
	endPointB[0] = value * cos(phiB);
	endPointB[1] = value * sin(phiB);
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

double LidarLine2::getPhiLow() const
{
	return min(phiA, phiB);
}

double LidarLine2::getPhiHigh() const
{
	return max(phiA, phiB);
}

void LidarLine2::transform(double angle, double tx, double ty)
{
	double c = cos(angle);
	double s = sin(angle);

	transform(angle, c, s, tx, ty);
}

void LidarLine2::transform(double angle, double c, double s, double tx, double ty)
{
	alpha += angle;
	l += tx * cos(alpha) + ty * sin(alpha);

	endPointA.transform2D(c, s, tx, ty);
	endPointB.transform2D(c, s, tx, ty);

	phiA = endPointA.getAngle2D();
	phiB = endPointB.getAngle2D();
}

double LidarLine2::error(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double l2 = l * l;

	double l1 = other.getL();
	double l12 = l1 * l1;
	double alpha1 = other.getAlpha();

	double one__sin = 2 * l * l1;
	double errLow = 0;
	double errHigh = 0;

	if(abs(alpha - alpha1) <= numeric_limits<double>::epsilon())
	{
		double tanLow = tan(phiLow - alpha);
		double tanHigh = tan(phiHigh - alpha);

		errLow = l2 * tanLow + l12 * tanLow - one__sin * tanLow;
		errHigh = l2 * tanHigh + l12 * tanHigh - one__sin * tanHigh;
	}
	else
	{
		one__sin /= sin(alpha - alpha1);
		errLow = l2 * tan(phiLow - alpha) + l12 * tan(phiLow - alpha1) - one__sin * log(cos(alpha - phiLow) / cos(alpha1 - phiLow));
		errHigh = l2 * tan(phiHigh - alpha) + l12 * tan(phiHigh - alpha1) - one__sin * log(cos(alpha - phiHigh) / cos(alpha1 - phiHigh));
	}

	return errHigh - errLow;
}

ostream& operator<<(ostream& out, const LidarLine2& lidarLine)
{
	return out << "LL(|" << lidarLine.l << "|, " << lidarLine.alpha << ", <" << lidarLine.phiA << ", " << lidarLine.phiB << ">)";
}

}}
