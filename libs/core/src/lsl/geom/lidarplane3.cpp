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

#include "lsl/geom/lidarplane3.hpp"

#include <iostream>
#include <limits>
#include <sstream>

#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace geom {

LidarPlane3::LidarPlane3(double l, const geom::Vector2d& normal, const geom::Vector2d& beginBound, const geom::Vector2d& endBound, int id) :
	l(l), normal(normal), id(id)
{
	setBounds(beginBound, endBound);
}

LidarPlane3::LidarPlane3(const Plane3& plane, const geom::Vector4d& beginBoundPoint, const geom::Vector4d& endBoundPoint, int id) :
	id(id)
{
	set(plane, beginBoundPoint, endBoundPoint);
}

LidarPlane3::LidarPlane3(const std::vector<Vector4d>& points, int id) :
	id(id)
{
	const geom::Vector4d& beginBoundPoint = points.front();
	const geom::Vector4d& endBoundPoint = points.back();

	set(Plane3::leastSquare(points, false), beginBoundPoint, endBoundPoint);
}

geom::Vector2d LidarPlane3::computeNormalAngles(const Plane3& plane) const
{
	geom::Vector2d normalAngles;
	computeNormalAngles(plane, normalAngles);

	return normalAngles;
}

void LidarPlane3::computeNormalAngles(const Plane3& plane, geom::Vector2d& normalAngles) const
{
	geom::Vector3d planeNormal = plane.getNormal();
	double normalNorm = planeNormal.norm();

	planeNormal *= -plane.getD();
	normalAngles[0] = planeNormal.getAngle2D();
	normalAngles[1] = utils::MathUtils::normAngle(std::acos(planeNormal[2] / normalNorm));
}

geom::Vector2d LidarPlane3::computeBound(const geom::Vector4d& boundPoint) const
{
	geom::Vector2d bound;
	bound[0] = boundPoint.getAngle2D();
	bound[1] = utils::MathUtils::normAngle(std::acos(boundPoint[2]));

	return bound;
}

void LidarPlane3::set(const Plane3& plane, const geom::Vector4d& beginBoundPoint, const geom::Vector4d& endBoundPoint)
{
	geom::Vector2d normalAngles = computeNormalAngles(plane);
	geom::Vector2d beginBound = computeBound(beginBoundPoint);
	geom::Vector2d endBound = computeBound(endBoundPoint);

	checkBounds(normalAngles, beginBound, endBound);

	geom::Vector3d planeNormal = plane.getNormal();
	l = std::abs(plane.getD() / planeNormal.norm());
	normal = normalAngles;

	setBounds(beginBound, endBound, false);
}

double LidarPlane3::getRawDistance(const geom::Vector2d& angles) const
{
	double c = std::cos(angles[0] - normal[0]);
	return 2 * l / ((c + 1) * std::cos(angles[1] - normal[1]) - (c - 1) * std::cos(angles[1] + normal[1]));
}

double LidarPlane3::getRawCloseness(const geom::Vector2d& angles) const
{
	double c = std::cos(angles[0] - normal[0]);
	return ((c + 1) * std::cos(angles[1] - normal[1]) - (c - 1) * std::cos(angles[1] + normal[1])) / (2 * l);
}

geom::Vector4d LidarPlane3::getRawPoint(const geom::Vector2d& angles) const
{
	double distance = getDistance(angles);
	double s = std::sin(angles[1]);

	geom::Vector4d point;
	point[0] = distance * std::cos(angles[0]) * s;
	point[1] = distance * std::sin(angles[0]) * s;
	point[2] = distance * std::cos(angles[1]);
	point[3] = 1;

	return point;
}

void LidarPlane3::setBounds(const geom::Vector2d& beginBound, const geom::Vector2d& endBound, bool check)
{
	if(check)
		checkBounds(normal, beginBound, endBound);

	this->beginBound = beginBound;
	this->endBound = endBound;

	beginBoundPoint = getRawPoint(beginBound);
	endBoundPoint = getRawPoint(endBound);
}

void LidarPlane3::checkBounds(const geom::Vector2d& normal, const geom::Vector2d& beginBound, const geom::Vector2d& endBound)
{
	geom::Vector2d beginDiffs = beginBound - normal;
	geom::Vector2d endDiffs = endBound - normal;

	for(int i = 0; i < beginDiffs.rows(); i++)
	{
		double beginDiff = utils::MathUtils::normAnglePi(beginDiffs[i]);
		double endDiff = utils::MathUtils::normAnglePi(endDiffs[i]);

		if(!(beginDiff > -utils::MathUtils::PI__TWO && beginDiff < utils::MathUtils::PI__TWO && endDiff > -utils::MathUtils::PI__TWO && endDiff < utils::MathUtils::PI__TWO))
		{
			std::ostringstream oss;
			oss << *this;

			// throw std::invalid_argument("Both bounds (" + std::to_string(phiA) + ", " + std::to_string(phiB) + ") are not in the domain (" + std::to_string(alpha) + " ± π/2) of line " + oss.str() + ".");
			throw std::invalid_argument("Both bounds are not in the domain of plane " + oss.str() + ".");
		}
	}
}

double LidarPlane3::getDistance(const geom::Vector2d& angles) const
{
	if(inBounds(angles))
		return getRawDistance(angles);
	else
		return std::numeric_limits<double>::max();
}

double LidarPlane3::getPlaneDistance(const geom::Vector2d& angles) const
{
	if(inDomain(angles))
		return getRawDistance(angles);
	else
		return std::numeric_limits<double>::max();
}

double LidarPlane3::getCloseness(const geom::Vector2d& angles) const
{
	if(inBounds(angles))
		return getRawCloseness(angles);
	else
		return std::numeric_limits<double>::max();
}

double LidarPlane3::getPlaneCloseness(const geom::Vector2d& angles) const
{
	if(inDomain(angles))
		return getRawCloseness(angles);
	else
		return std::numeric_limits<double>::max();
}

geom::Vector4d LidarPlane3::getPoint(const geom::Vector2d& angles) const
{
	if(inBounds(angles))
		return getRawPoint(angles);
	else
		return geom::Vector4d::Zero();
}

geom::Vector4d LidarPlane3::getPlanePoint(const geom::Vector2d& angles) const
{
	if(inDomain(angles))
		return getRawPoint(angles);
	else
		return geom::Vector4d::Zero();
}

double LidarPlane3::getLowBound(std::size_t index) const
{
	return std::min(beginBound[index], endBound[index]);
}

geom::Vector2d LidarPlane3::getLowBound() const
{
	geom::Vector2d lowBound;
	for(int i = 0; i < normal.rows(); i++)
		lowBound[i] = getLowBound(i);

	return lowBound;
}

double LidarPlane3::getHighBound(std::size_t index) const
{
	return std::max(beginBound[index], endBound[index]);
}

geom::Vector2d LidarPlane3::getHighBound() const
{
	geom::Vector2d highBound;
	for(int i = 0; i < normal.rows(); i++)
		highBound[i] = getHighBound(i);

	return highBound;
}

double LidarPlane3::getLowDomain(std::size_t index) const
{
	return normal[index] - utils::MathUtils::PI__TWO;
}

geom::Vector2d LidarPlane3::getLowDomain() const
{
	geom::Vector2d lowDomain;
	for(int i = 0; i < normal.rows(); i++)
		lowDomain[i] = getLowDomain(i);

	return lowDomain;
}

double LidarPlane3::getHighDomain(std::size_t index) const
{
	return normal[index] + utils::MathUtils::PI__TWO;
}

geom::Vector2d LidarPlane3::getHighDomain() const
{
	geom::Vector2d highDomain;
	for(int i = 0; i < normal.rows(); i++)
		highDomain[i] = getHighDomain(i);

	return highDomain;
}

bool LidarPlane3::isVisible() const
{
	for(int i = 0; i < normal.rows(); i++)
		if(beginBound[i] >= endBound[i])
			return false;

	return true;
}

bool LidarPlane3::inBounds(const geom::Vector2d& angles) const
{
	for(int i = 0; i < normal.rows(); i++)
		if(angles[i] < getLowBound(i) || angles[i] > getHighBound(i))
			return false;

	return true;
}

bool LidarPlane3::inDomain(const geom::Vector2d& angles) const
{
	for(int i = 0; i < normal.rows(); i++)
		if(angles[i] < getLowDomain(i) || angles[i] > getHighDomain(i))
			return false;

	return true;
}

void LidarPlane3::transform(const Transformation& transformation)
{
}

void LidarPlane3::transformAll(std::vector<LidarPlane3>& lidarPlanes, const Transformation& transformation, bool removeInvalid)
{
}

void LidarPlane3::transformAllToLocation(std::vector<LidarPlane3>& lidarPlanes, const Location& location, bool removeInvalid)
{
	transformAll(lidarPlanes, createTransformation(location), removeInvalid);
}

double LidarPlane3::error(const LidarPlane3& other, const geom::Vector2d& lowBound, const geom::Vector2d& highBound) const
{
	double a = std::sin(highBound[0] - normal[0]) - std::sin(lowBound[0] - normal[0]);
	double a_ = std::sin(highBound[0] - other.normal[0]) - std::sin(lowBound[0] - other.normal[0]);

	double a2 = utils::intCos2(highBound[0], -normal[0]) - utils::intCos2(lowBound[0], -normal[0]);
	double b2 = utils::intCos2(highBound[1], -normal[1]) - utils::intCos2(lowBound[1], -normal[1]);
	double c2 = utils::intCos2(highBound[1], normal[1]) - utils::intCos2(lowBound[1], normal[1]);
	double bc = utils::intCos2(highBound[1], -normal[1], normal[1]) - utils::intCos2(lowBound[1], -normal[1], normal[1]);

	double a_2 = utils::intCos2(highBound[0], -other.normal[0]) - utils::intCos2(lowBound[0], -other.normal[0]);
	double b_2 = utils::intCos2(highBound[1], -other.normal[1]) - utils::intCos2(lowBound[1], -other.normal[1]);
	double c_2 = utils::intCos2(highBound[1], other.normal[1]) - utils::intCos2(lowBound[1], other.normal[1]);
	double b_c_ = utils::intCos2(highBound[1], -other.normal[1], other.normal[1]) - utils::intCos2(lowBound[1], -other.normal[1], other.normal[1]);

	double aa_ = utils::intCos2(highBound[0], -normal[0], -other.normal[0]) - utils::intCos2(lowBound[0], -normal[0], -other.normal[0]);
	double bb_ = utils::intCos2(highBound[1], -normal[1], -other.normal[1]) - utils::intCos2(lowBound[1], -normal[1], -other.normal[1]);
	double cc_ = utils::intCos2(highBound[1], normal[1], other.normal[1]) - utils::intCos2(lowBound[1], normal[1], other.normal[1]);
	double bc_ = utils::intCos2(highBound[1], -normal[1], other.normal[1]) - utils::intCos2(lowBound[1], -normal[1], other.normal[1]);
	double cb_ = utils::intCos2(highBound[1], normal[1], -other.normal[1]) - utils::intCos2(lowBound[1], normal[1], -other.normal[1]);

	double p2 = (a2 * b2 + 2 * a * b2 + b2 - 2 * a2 * bc + 2 * bc + a2 * c2 - 2 * a * c2 + c2) / (4 * l * l);
	double p_2 = (a_2 * b_2 + 2 * a_ * b_2 + b_2 - 2 * a_2 * b_c_ + 2 * b_c_ + a_2 * c_2 - 2 * a_ * c_2 + c_2) / (4 * other.l * other.l);
	double pp_ = (aa_ * bb_ + a * bb_ - aa_ * bc_ + a * bc_ + a_ * bb_ + bb_ - a_ * bc_ + bc_ - aa_ * cb_ - a * cb_ + aa_ * cc_ - a * cc_ + a_ * cb_ + cb_ - a_ * cc_ + cc_) / (2 * l * other.l);

	return p2 - pp_ + p_2;
}

LidarPlane3::Location LidarPlane3::gradientErrorAtZero(const LidarPlane3& other, const geom::Vector2d& lowBound, const geom::Vector2d& highBound) const
{
	Location gradient = Location::Zero();
	return gradient;
}

bool LidarPlane3::operator==(const LidarPlane3& other) const
{
	return (l == other.l && normal == other.normal && beginBound == other.beginBound && endBound == other.endBound);
}

std::ostream& operator<<(std::ostream& out, const LidarPlane3& lidarPlane)
{
	return out << "LP(|" << lidarPlane.l << "|, (" << lidarPlane.normal << "), <" << lidarPlane.beginBound << ", " << lidarPlane.endBound << ">)";
}

}}
