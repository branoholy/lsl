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

#include "lsl/registration/icp.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace registration {

ICP::ICP() : Registration(),
	maxIterations(50),
	minFinalError(0.0001),
	maxDistance2(1),
	finalError(0),
	finalTransformation(PointCloudType::Transformation::Identity())
{
}

void ICP::loadConfig(const std::string& path)
{
	auto config = YAML::LoadFile(path);
	auto yamlCorrespondences = config["correspondences"];
	auto yamlMinimization = config["minimization"];

	if(yamlCorrespondences)
	{
		if(yamlCorrespondences["maxDistance"]) maxDistance2 = std::pow(yamlCorrespondences["maxDistance"].as<double>(), 2);
	}

	if(yamlMinimization)
	{
		if(yamlMinimization["maxIterations"]) maxIterations = yamlMinimization["maxIterations"].as<std::size_t>();
		if(yamlMinimization["minFinalError"]) minFinalError = yamlMinimization["minFinalError"].as<double>();
	}
}

void ICP::setTarget(const PointCloudType& target)
{
	this->target = target;
}

void ICP::setSource(const PointCloudType& source, bool setOldSourceAsTarget)
{
	if(setOldSourceAsTarget) target = std::move(this->source);
	this->source = source;
}

void ICP::alignSteps(std::size_t steps)
{
	align(target, source, steps);
}

void ICP::alignSteps(const PointCloudType::Transformation& guess, std::size_t steps)
{
	PointCloudType sourceT = source;
	sourceT.transform(guess);

	align(target, sourceT, steps);
	finalTransformation = finalTransformation * guess;
}

void ICP::align()
{
	align(target, source, maxIterations);
}

void ICP::align(const PointCloudType::Transformation& guess)
{
	alignSteps(guess, maxIterations);
}

void ICP::align(const PointCloudType& target, const PointCloudType& source, std::size_t maxIterations)
{
	PointCloudType sourceT = source;
	std::vector<std::size_t> matches;

	converged = false;
	double oldError = std::numeric_limits<double>::max();
	PointCloudType::Transformation transformation = PointCloudType::Transformation::Identity();

	for(iterationCount = 0; iterationCount < maxIterations; iterationCount++)
	{
		PointCloudType::Transformation T = PointCloudType::Transformation::Identity();
		if(iterationCount > 0)
		{
			T = computeTransformation(target, sourceT, matches);
			sourceT.transform(T);
		}

		double error = matchPoints(target, sourceT, matches);
		if(error < oldError) transformation = T * transformation;

		if(error < minFinalError || error >= oldError)
		{
			if(error < minFinalError) finalError = error;
			else finalError = oldError;

			converged = true;
			break;
		}

		oldError = error;
	}

	if(!converged) finalError = oldError;

	finalTransformation = transformation;

	evaluationCount = iterationCount;
}

double ICP::matchPoints(const PointCloudType& target, const PointCloudType& source, std::vector<std::size_t>& matches)
{
	matches.clear();
	matches.resize(target.size());
	std::fill(matches.begin(), matches.end(), -1);

	double *distances2 = utils::ArrayUtils::createFilledArray(target.size(), std::numeric_limits<double>::max());

	for(std::size_t i = 0; i < source.size(); i++)
	{
		double minDistance2 = std::numeric_limits<double>::max();
		int minIndex = -1;

		for(std::size_t j = 0; j < target.size(); j++)
		{
			double distance2 = (target[i] - source[j]).squaredNorm();
			if(distance2 < maxDistance2 && distance2 < minDistance2)
			{
				minDistance2 = distance2;
				minIndex = j;
			}
		}

		if(minDistance2 < distances2[minIndex])
		{
			matches[minIndex] = i;
			distances2[minIndex] = minDistance2;
		}
	}

	double error = 0;
	for(std::size_t i = 0; i < target.size(); i++)
	{
		if(matches[i] != std::size_t(-1)) error += distances2[i];
	}
	delete[] distances2;

	return error;
}

ICP::PointCloudType::Transformation ICP::computeTransformation(const PointCloudType& target, const PointCloudType& source, const std::vector<std::size_t>& matches)
{
	PointCloudType::Point targetCentroid = PointCloudType::Point::Zero();
	PointCloudType::Point sourceCentroid = PointCloudType::Point::Zero();

	std::size_t matchesSize = 0;
	for(std::size_t i = 0; i < target.size(); i++)
	{
		if(matches[i] == std::size_t(-1)) continue;

		targetCentroid += target[i];
		sourceCentroid += source[matches[i]];
		matchesSize++;
	}
	targetCentroid *= 1.0 / matchesSize;
	sourceCentroid *= 1.0 / matchesSize;

	geom::Matrix<PointCloudType::ScalarType, 2, 2> H = geom::Matrix<PointCloudType::ScalarType, 2, 2>::Zero();
	for(std::size_t i = 0; i < target.size(); i++)
	{
		if(matches[i] == std::size_t(-1)) continue;

		PointCloudType::Point targetPoint = target[i] - targetCentroid;
		PointCloudType::Point sourcePoint = source[matches[i]] - sourceCentroid;

		H += (sourcePoint.head<2>() * targetPoint.head<2>().transpose());
	}

	Eigen::JacobiSVD<geom::Matrix<PointCloudType::ScalarType, 2, 2>> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	geom::Matrix<PointCloudType::ScalarType, 2, 2> R = svd.matrixV() * svd.matrixU().transpose();

	double det = R.determinant();
	if(std::abs(det + 1) < lsl::utils::MathUtils::EPSILON_10)
	{
		geom::Matrix<PointCloudType::ScalarType, 2, 2> detM;
		detM << 1, 0,
				0, det;

		R = svd.matrixU() * detM * svd.matrixV().transpose();
	}

	geom::Matrix<PointCloudType::ScalarType, 2, 1> t = targetCentroid.head<2>() - R * sourceCentroid.head<2>();

	PointCloudType::Transformation T;
	T << R(0, 0), R(0, 1), t[0],
		 R(1, 0), R(1, 1), t[1],
		 0, 0, 1;

	return T;
}

}}
