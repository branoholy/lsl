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

#include "lsl/geom/lidarline2.hpp"

#include <iostream>
#include <limits>
#include <sstream>

#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace geom {

LidarLine2::LidarLine2(double l, double alpha, double phiA, double phiB, bool managed) :
	l(l), alpha(alpha), managed(managed)
{
	setPhiAB(phiA, phiB);
}

LidarLine2::LidarLine2(const Line2& line, const Vector3d& endPointA, const Vector3d& endPointB, bool managed) :
	managed(managed)
{
	set(line, endPointA, endPointB);
}

LidarLine2::LidarLine2(const Vector3d& endPointA, const Vector3d& endPointB, bool managed) : LidarLine2(Line2(endPointA, endPointB), endPointA, endPointB, managed)
{
}

LidarLine2::LidarLine2(const std::vector<Vector3d>& points, bool managed) :
	managed(managed)
{
	Line2 line = Line2::leastSquareLine(points);

	std::size_t indexEndPointA = 0, indexEndPointB = 0;
	for(std::size_t i = 1; i < points.size(); i++)
	{
		if(points[i].getId() < points[indexEndPointA].getId()) indexEndPointA = i;
		if(points[i].getId() > points[indexEndPointB].getId()) indexEndPointB = i;
	}

	Vector3d endPointA = points[indexEndPointA];
	Vector3d endPointB = points[indexEndPointB];
	bool visibility = endPointA.getAngle2D() < endPointB.getAngle2D();

	Vector3d closestEndPointA = line.getClosestPoint(points[indexEndPointA]);
	Vector3d closestEndPointB = line.getClosestPoint(points[indexEndPointB]);
	bool closestVisibility = closestEndPointA.getAngle2D() < closestEndPointB.getAngle2D();
	if(visibility == closestVisibility)
	{
		endPointA = std::move(closestEndPointA);
		endPointB = std::move(closestEndPointB);
	}

	set(line, endPointA, endPointB);
}

void LidarLine2::setLine(const Line2& line)
{
	Vector2d normal = line.getNormal();
	l = abs(line.getC() / normal.norm());

	normal *= -line.getC();
	alpha = normal.getAngle2D();
}

void LidarLine2::set(const Line2& line)
{
	testBounds(line, endPointA, endPointB);
	setLine(line);
}

void LidarLine2::set(const Line2& line, const Vector3d& endPointA, const Vector3d& endPointB)
{
	testBounds(line, endPointA, endPointB);

	setLine(line);
	setEndPointA(endPointA, false);
	setEndPointB(endPointB, false);
}

double LidarLine2::getValue(double phi) const
{
	if(inBounds(phi)) return l / std::cos(phi - alpha);
	else return std::numeric_limits<double>::max();
}

double LidarLine2::getLineValue(double phi) const
{
	if(inDomain(phi)) return l / std::cos(phi - alpha);
	else return std::numeric_limits<double>::max();
}

void LidarLine2::setPhiA(double phiA)
{
	testBounds(alpha, phiA, phiB);

	this->phiA = phiA;
	setEndPointA(phiA);
}

void LidarLine2::setPhiB(double phiB)
{
	testBounds(alpha, phiA, phiB);

	this->phiB = phiB;
	setEndPointB(phiB);
}

void LidarLine2::setPhiAB(double phiA, double phiB)
{
	testBounds(alpha, phiA, phiB);

	this->phiA = phiA;
	this->phiB = phiB;

	setEndPointA(phiA);
	setEndPointB(phiB);
}

void LidarLine2::setEndPointA(double phiA)
{
	double value = getValue(phiA);
	endPointA[0] = value * cos(phiA);
	endPointA[1] = value * sin(phiA);
	endPointA[2] = 1;
}

void LidarLine2::setEndPointB(double phiB)
{
	double value = getValue(phiB);
	endPointB[0] = value * cos(phiB);
	endPointB[1] = value * sin(phiB);
	endPointB[2] = 1;
}

void LidarLine2::setEndPointA(const Vector3d& endPointA)
{
	setEndPointA(endPointA, true);
}

void LidarLine2::setEndPointA(const Vector3d& endPointA, bool testBounds_)
{
	double phiA = endPointA.getAngle2D();
	if(testBounds_) testBounds(alpha, phiA, phiB);

	this->endPointA = endPointA;
	this->phiA = phiA;
}

void LidarLine2::setEndPointB(const Vector3d& endPointB)
{
	setEndPointB(endPointB, true);
}

void LidarLine2::setEndPointB(const Vector3d& endPointB, bool testBounds_)
{
	double phiB = endPointB.getAngle2D();
	if(testBounds_) testBounds(alpha, phiA, phiB);

	this->endPointB = endPointB;
	this->phiB = phiB;
}

double LidarLine2::getPhiLow() const
{
	return std::min(phiA, phiB);
}

double LidarLine2::getPhiHigh() const
{
	return std::max(phiA, phiB);
}

double LidarLine2::getDomainLow() const
{
	return alpha - utils::MathUtils::PI__TWO;
}

double LidarLine2::getDomainHigh() const
{
	return alpha + utils::MathUtils::PI__TWO;
}

bool LidarLine2::isVisible() const
{
	return phiA < phiB;
}

bool LidarLine2::inBounds(double phi) const
{
	return phi >= getPhiLow() && phi <= getPhiHigh();
}

bool LidarLine2::inDomain(double phi) const
{
	double phi0 = utils::MathUtils::normAnglePi(phi - alpha);
	return phi0 > -utils::MathUtils::PI__TWO && phi0 < utils::MathUtils::PI__TWO;
}

void LidarLine2::testBounds(double alpha, double phiA, double phiB)
{
	if(!managed) return;

	double phiA0 = utils::MathUtils::normAnglePi(phiA - alpha);
	double phiB0 = utils::MathUtils::normAnglePi(phiB - alpha);

	if(!(phiA0 > -utils::MathUtils::PI__TWO && phiA0 < utils::MathUtils::PI__TWO && phiB0 > -utils::MathUtils::PI__TWO && phiB0 < utils::MathUtils::PI__TWO))
	{
		std::ostringstream oss;
		oss << *this;

		throw std::invalid_argument("Both bounds (" + std::to_string(phiA) + ", " + std::to_string(phiB) + ") are not in the domain (" + std::to_string(alpha) + " ± π/2) of line " + oss.str() + ".");
	}
}

void LidarLine2::testBounds(const Line2& line, const Vector3d& endPointA, const Vector3d& endPointB)
{
	if(!managed) return;

	Vector2d normal = line.getNormal();
	normal *= -line.getC();

	double alpha = normal.getAngle2D();
	double phiA = endPointA.getAngle2D();
	double phiB = endPointB.getAngle2D();

	testBounds(alpha, phiA, phiB);
}

bool LidarLine2::checkBounds() const
{
	return inDomain(phiA) && inDomain(phiB);
}

void LidarLine2::transform(const Transformation& transformation)
{
	double alpha = this->alpha + std::atan2(transformation(1, 0), transformation(0, 0));
	Vector3d endPointA = transformation * this->endPointA;
	Vector3d endPointB = transformation * this->endPointB;

	double phiA = endPointA.getAngle2D();
	double phiB = endPointB.getAngle2D();
	testBounds(alpha, phiA, phiB);

	this->alpha = alpha;
	l += transformation(0, 2) * std::cos(this->alpha) + transformation(1, 2) * std::sin(this->alpha);

	this->endPointA = std::move(endPointA);
	this->endPointB = std::move(endPointB);

	this->phiA = phiA;
	this->phiB = phiB;

	if(l < 0)
	{
		this->alpha = utils::MathUtils::normAngle(this->alpha - utils::MathUtils::PI);
		l *= -1;
	}
}

void LidarLine2::transformAll(std::vector<LidarLine2>& lidarLines, const Transformation& transformation, bool removeInvalid)
{
	std::vector<LidarLine2> linesToAdd;
	for(size_t i = 0; i < lidarLines.size(); i++)
	{
		LidarLine2& line = lidarLines[i];

		try
		{
			line.transform(transformation);
		}
		catch(const std::invalid_argument&)
		{
			if(removeInvalid)
			{
				lidarLines.erase(lidarLines.begin() + i);
				i--;
				continue;
			}
			else throw;
		}

		// Divide lines crossing x+ axis (could happened only after transformation)
		double phiL = line.getPhiLow();
		double phiH = line.getPhiHigh();

		double phiDiff = phiH - phiL;
		if(phiDiff > utils::MathUtils::PI)
		{
			LidarLine2 lineDown = line;
			if(phiL == line.getPhiA())
			{
				line.setPhiB(0);
				lineDown.setPhiA(utils::MathUtils::TWO_PI_EXCLUSIVE);
			}
			else
			{
				line.setPhiA(0);
				lineDown.setPhiB(utils::MathUtils::TWO_PI_EXCLUSIVE);
			}

			linesToAdd.push_back(lineDown);
		}
	}

	lidarLines.insert(lidarLines.end(), linesToAdd.begin(), linesToAdd.end());
}

void LidarLine2::transformAllToLocation(std::vector<LidarLine2>& lidarLines, const Location& location, bool removeInvalid)
{
	transformAll(lidarLines, createTransformation(location), removeInvalid);
}

double LidarLine2::error(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double l2 = l * l;

	double l_ = other.getL();
	double l_2 = l_ * l_;
	double alpha_ = other.getAlpha();

	double one__sin = 2 * l * l_;
	double errLow = 0;
	double errHigh = 0;

	double err;
	if(std::abs(utils::MathUtils::normAnglePi(alpha - alpha_)) <= utils::MathUtils::EPSILON_10)
	{
		double tanLow = std::tan(phiLow - alpha);
		double tanHigh = std::tan(phiHigh - alpha);

		errLow = l2 * tanLow + l_2 * tanLow - one__sin * tanLow;
		errHigh = l2 * tanHigh + l_2 * tanHigh - one__sin * tanHigh;

		err = errHigh - errLow;
		if(err < 0) std::cerr << "---> " << err << std::endl;
		if(err < 0 && std::abs(err) <= utils::MathUtils::EPSILON_10) err = 0;
	}
	else
	{
		one__sin /= std::sin(alpha - alpha_);
		errLow = l2 * std::tan(phiLow - alpha) + l_2 * std::tan(phiLow - alpha_) - one__sin * std::log(cos(alpha - phiLow) / std::cos(alpha_ - phiLow));
		errHigh = l2 * std::tan(phiHigh - alpha) + l_2 * std::tan(phiHigh - alpha_) - one__sin * std::log(cos(alpha - phiHigh) / std::cos(alpha_ - phiHigh));

		err = errHigh - errLow;
		if(err < 0) std::cerr << "->>> " << err << ", " << utils::MathUtils::normAngle(alpha - alpha_) << std::endl;
	}

	return err;
}

Vector2d LidarLine2::gradientTranslationAtZero(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double l_ = other.getL();
	double alpha_ = other.getAlpha();

	double pLow, pHigh;
	if(std::abs(utils::MathUtils::normAnglePi(alpha - alpha_)) <= utils::MathUtils::EPSILON_10)
	{
		pLow = std::tan(phiLow - alpha);
		pHigh = std::tan(phiHigh - alpha);
	}
	else
	{
		double one__sin = 1.0 / std::sin(alpha - alpha_);
		pLow = one__sin * std::log(std::cos(alpha - phiLow) / std::cos(alpha_ - phiLow));
		pHigh = one__sin * std::log(std::cos(alpha - phiHigh) / std::cos(alpha_ - phiHigh));
	}

	double tanP = (l_ * std::tan(phiHigh - alpha_) - l * pHigh) - (l_ * std::tan(phiLow - alpha_) - l * pLow);

	Vector2d gradient;
	gradient[0] = 2 * std::cos(alpha_) * tanP;
	gradient[1] = 2 * std::sin(alpha_) * tanP;

	return gradient;
}

double LidarLine2::gradientRotationAtZero(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double l_ = other.getL();
	double alpha_ = other.getAlpha();

	double gradientPhi;
	if(std::abs(utils::MathUtils::normAnglePi(alpha - alpha_)) <= utils::MathUtils::EPSILON_10)
	{
		double one__cosLow = 1 / std::cos(phiLow - alpha_);
		double one__cosHigh = 1 / std::cos(phiHigh - alpha_);

		gradientPhi = -l_ * l_ * one__cosHigh * one__cosHigh + 2 * l * l_ * one__cosHigh * one__cosHigh;
		gradientPhi -= -l_ * l_ * one__cosLow * one__cosLow + 2 * l * l_ * one__cosLow * one__cosLow;
	}
	else
	{
		gradientPhi = -2 * l * l_ * utils::csc(alpha - alpha_) * std::tan(alpha_ - phiHigh) - 2 * l * l_ * utils::cot(alpha - alpha_) * utils::csc(alpha - alpha_) * std::log(std::cos(alpha - phiHigh) * utils::sec(alpha_ - phiHigh)) - l_ * l_ * utils::sec2(alpha_ - phiHigh);
		gradientPhi -= -2 * l * l_ * utils::csc(alpha - alpha_) * std::tan(alpha_ - phiLow) - 2 * l * l_ * utils::cot(alpha - alpha_) * utils::csc(alpha - alpha_) * std::log(std::cos(alpha - phiLow) * utils::sec(alpha_ - phiLow)) - l_ * l_ * utils::sec2(alpha_ - phiLow);
	}

	return gradientPhi;
}

Vector3d LidarLine2::gradientErrorAtZero(const LidarLine2& other, double phiLow, double phiHigh) const
{
	Vector2d translationGradient = gradientTranslationAtZero(other, phiLow, phiHigh);
	double rotationGradient = gradientRotationAtZero(other, phiLow, phiHigh);

	return Vector3d(translationGradient[0], translationGradient[1], rotationGradient);
}

Vector2d LidarLine2::gradientTranslationNormAtZero(const LidarLine2& other, double phiLow, double phiHigh) const
{
	double alpha_ = other.getAlpha();
	double cosAlpha_ = std::cos(alpha_);
	double sinAlpha_ = std::sin(alpha_);
	double tanDiff = std::tan(phiHigh - alpha_) - std::tan(phiLow - alpha_);

	Vector2d gradientNorm;

	gradientNorm[0] = 2 * cosAlpha_ * cosAlpha_ * tanDiff;
	gradientNorm[1] = 2 * sinAlpha_ * sinAlpha_ * tanDiff;

	return gradientNorm;
}

bool LidarLine2::operator==(const LidarLine2& other) const
{
	return (l == other.l && alpha == other.alpha && phiA == other.phiA && phiB == other.phiB);
}

double LidarLine2::sumDf(const std::vector<LidarLine2>& lidarLines)
{
	double sum = 0;
	for(const LidarLine2& line : lidarLines)
	{
		sum += line.getPhiHigh() - line.getPhiLow();
	}

	return sum;
}

std::ostream& operator<<(std::ostream& out, const LidarLine2& lidarLine)
{
	return out << "LL(|" << lidarLine.l << "|, " << lidarLine.alpha << ", <" << lidarLine.phiA << ", " << lidarLine.phiB << ">)";
}

}}
