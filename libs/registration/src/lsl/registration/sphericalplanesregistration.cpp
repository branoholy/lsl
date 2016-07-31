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

#include "lsl/registration/sphericalplanesregistration.hpp"

#include <set>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace registration {

void SphericalPlanesRegistration::loadConfig(const std::string& path)
{
}

void SphericalPlanesRegistration::setTarget(const PointCloudType& target)
{
	targetPlanes = detectPlanes(target, maxPlaneCount);
}

void SphericalPlanesRegistration::setSource(const PointCloudType& source, bool setOldSourceAsTarget)
{
	setSourcePlanes(detectPlanes(source, maxPlaneCount), setOldSourceAsTarget);
}

void SphericalPlanesRegistration::setTargetPlanes(const std::vector<geom::LidarPlane3>& targetPlanes)
{
	this->targetPlanes = targetPlanes;
}

void SphericalPlanesRegistration::setSourcePlanes(const std::vector<geom::LidarPlane3>& sourcePlanes, bool setOldSourceAsTarget)
{
	if(setOldSourceAsTarget) targetPlanes = std::move(this->sourcePlanes);
	this->sourcePlanes = sourcePlanes;
}

std::vector<geom::LidarPlane3> SphericalPlanesRegistration::detectPlanes(const PointCloudType& points, std::size_t maxPlaneCount) const
{
	if(maxPlaneCount == std::size_t(-1)) maxPlaneCount = this->maxPlaneCount;

	std::vector<geom::LidarPlane3> planes;
/*
	if(detectionTransformation.isIdentity()) planes = ransac.run<geom::LidarPlane3>(points, maxPlaneCount);
	else
	{
		PointCloudType pointsT = points;
		pointsT.transform(detectionTransformation);

		planes = ransac.run<geom::LidarPlane3>(pointsT, maxPlaneCount);
	}

	if(!detectionTransformation.isIdentity())
	{
		lsl::geom::LidarPlane3::transformAll(planes, detectionTransformation.transpose(), true);
	}

	removeInvisible(planes);
*/

	return planes;
}

void SphericalPlanesRegistration::removeInvisible(std::vector<geom::LidarPlane3>& planes) const
{
	if(planes.empty()) return;

}
/*
double LLR::getAvgDiffL(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const
{
	double sumDiffL = 0;
	std::size_t countDiffL = 0;
	iterAllLines(targetLines, sourceLines, [&sumDiffL, &countDiffL] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double, double)
	{
		sumDiffL += std::abs(targetLine.getL() - sourceLine.getL());
		countDiffL++;
	});

	return sumDiffL / countDiffL;
}
*/
void SphericalPlanesRegistration::iterBounds(const std::multiset<Bound>& bounds, iterBoundsFunc f) const
{
	std::vector<const Bound*> buffers[2];
	std::vector<const Bound*> tmpBuffers[2];
	for(auto it = bounds.begin(); it != bounds.end(); it++)
	{
		const Bound& bound = *it;

		if(it != bounds.begin() && (*std::prev(it)).value != bound.value)
		{
			for(std::size_t i = 0; i < 2; i++)
				tmpBuffers[i].clear();
		}

		if(bound.beginBound)
		{
			buffers[bound.scanId].push_back(&bound);
			tmpBuffers[bound.scanId].push_back(&bound);
		}
		else if(!buffers[bound.scanId].empty())
		{
			auto beginBound = std::find_if(buffers[bound.scanId].begin(), buffers[bound.scanId].end(), [&bound](const Bound *other)
			{
				return (bound.scanId == other->scanId && bound.index == other->index && bound.beginBound != other->beginBound);
			});

			buffers[bound.scanId].erase(beginBound);
		}

		auto nextIt = std::next(it);
		if(nextIt != bounds.end() && (*nextIt).value == bound.value)
			continue;

		if(tmpBuffers[bound.scanId].size() * buffers[(bound.scanId + 1) % 2].size() > 0)
		{
			std::vector<const Bound*> *targetBounds, *sourceBounds;
			if(bound.scanId == 0)
			{
				targetBounds = &tmpBuffers[0];
				sourceBounds = &buffers[1];
			}
			else
			{
				targetBounds = &buffers[0];
				sourceBounds = &tmpBuffers[1];
			}

			f(*targetBounds, *sourceBounds);
		}
	}
}

std::size_t SphericalPlanesRegistration::iterAllPlanes(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, iterPlanesFunc f) const
{
	return iterPlanes(targetPlanes, sourcePlanes, f, false);
}

std::size_t SphericalPlanesRegistration::iterPlanes(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, iterPlanesFunc f, bool onlyCorresponding) const
{
	std::multiset<Bound> bounds[2];
	for(std::size_t i = 0; i < 2; i++)
	{
		std::size_t index = 0;
		for(const geom::LidarPlane3& plane : targetPlanes)
		{
			bounds[i].insert(Bound(plane.getBeginBound()[i], 0, true, index));
			bounds[i].insert(Bound(plane.getEndBound()[i], 0, false, index));
			index++;
		}

		index = 0;
		for(const geom::LidarPlane3& plane : sourcePlanes)
		{
			bounds[i].insert(Bound(plane.getBeginBound()[i], 1, true, index));
			bounds[i].insert(Bound(plane.getEndBound()[i], 1, false, index));
			index++;
		}
	}

	std::size_t betterIndex, betterSum = -1;
	for(std::size_t i = 0; i < 2; i++)
	{
		std::size_t sum = 0;
		iterBounds(bounds[i], [&sum](const std::vector<const Bound*>& targetBounds, const std::vector<const Bound*>& sourceBounds)
		{
			sum += targetBounds.size() * sourceBounds.size();
		});

		if(sum < betterSum)
		{
			betterIndex = i;
			betterSum = sum;
		}
	}

	std::size_t counter = 0;
	iterBounds(bounds[betterIndex], [&counter, &targetPlanes, &sourcePlanes, &f](const std::vector<const Bound*>& targetBounds, const std::vector<const Bound*>& sourceBounds)
	{
		for(const Bound *targetBound : targetBounds)
		{
			geom::LidarPlane3 targetPlane = targetPlanes.at(targetBound->index);

			for(const Bound *sourceBound : sourceBounds)
			{
				bool callF = true;
				geom::LidarPlane3 sourcePlane = sourcePlanes.at(sourceBound->index);

				geom::Vector2d beginBound;
				geom::Vector2d endBound;
				for(std::size_t i = 0; i < 2; i++)
				{
					beginBound[i] = std::max(targetPlane.getBeginBound()[i], sourcePlane.getBeginBound()[i]);
					endBound[i] = std::min(targetPlane.getEndBound()[i], sourcePlane.getEndBound()[i]);

					if(beginBound[i] >= endBound[i])
						callF = false;
				}

				if(callF)
					f(counter++, targetPlane, sourcePlane, beginBound, endBound);
			}
		}
	});

	/*
	while(i1 < size1 && i2 < size2)
	{
		const geom::LidarLine2& targetLine = targetLines.at(i1);
		const geom::LidarLine2& sourceLine = sourceLines.at(i2);

		double maxPhiA = std::max(targetLine.getPhiA(), sourceLine.getPhiA());
		double minPhiB = std::min(targetLine.getPhiB(), sourceLine.getPhiB());

		if(maxPhiA < minPhiB)
		{
			bool callF = !onlyCorresponding;
			if(onlyCorresponding)
			{
				double diffL = std::abs(targetLine.getL() - sourceLine.getL());
				callF = (diffL < maxDiffL) && (diffL < (avgDiffL + maxAvgDiffL));
			}

			if(callF) f(counter++, targetLine, sourceLine, maxPhiA, minPhiB);
		}

		if(minPhiB >= targetLine.getPhiB()) i1++;
		if(minPhiB >= sourceLine.getPhiB()) i2++;
	}
	*/

	return counter;
}

double SphericalPlanesRegistration::error(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes)
{
	sumOfErrors = 0;
	// coverAngleFactor = 0;

	iterPlanes(targetPlanes, sourcePlanes, [this] (std::size_t, const geom::LidarPlane3& targetPlane, const geom::LidarPlane3& sourcePlane, const geom::Vector2d& topLeft, const geom::Vector2d& bottomRight)
	{
		double ei = 0; // targetPlane.error(sourcePlane, phiA, phiB);

		if(ei >= 0) // TODO: Is it needed?
		{
			sumOfErrors += ei;
			// coverAngleFactor += phiB - phiA;
		}
		else
		{
			std::cerr << "ei < 0: " << topLeft << '-' << bottomRight << ": " << targetPlane << ' ' << sourcePlane << std::endl;
		}
	});

	finalError = sumOfErrors;
	/*
	finalError = coverFactor = std::numeric_limits<double>::max();
	if(coverAngleFactor > 0)
	{
		double one__caf = 1 / coverAngleFactor;
		coverFactor = one__caf * utils::MathUtils::TWO_PI;
		finalError = coverFactor * one__caf * sumOfErrors;
	}
	*/

	return finalError;
}

double SphericalPlanesRegistration::errorTransform(const std::vector<geom::LidarPlane3>& targetPlanes, std::vector<geom::LidarPlane3> sourcePlanes, PointCloudType::Transformation transformation)
{
	geom::LidarPlane3::transformAll(sourcePlanes, transformation, true);
	removeInvisible(sourcePlanes);

	return error(targetPlanes, sourcePlanes);
}

SphericalPlanesRegistration::PointCloudType SphericalPlanesRegistration::errorAreas(const std::vector<geom::LidarPlane3>& targetPlanes, std::vector<geom::LidarPlane3> sourcePlanes, PointCloudType::Transformation transformation) const
{
	geom::LidarPlane3::transformAll(sourcePlanes, transformation, true);
	removeInvisible(sourcePlanes);

	PointCloudType areas;
	iterPlanes(targetPlanes, sourcePlanes, [&areas] (std::size_t, const geom::LidarPlane3& targetPlane, const geom::LidarPlane3& sourcePlane, const geom::Vector2d& topLeft, const geom::Vector2d& bottomRight)
	{
		double ei = 0; // targetPlane.error(sourcePlane, phiA, phiB);
		if(ei >= 0) // TODO: Is it needed?
		{
			/*
			double coses[] = { std::cos(phiA), std::cos(phiB) };
			double sins[] = { std::sin(phiA), std::sin(phiB) };
			double values[] = { targetLine.getValue(phiA), sourceLine.getValue(phiA), sourceLine.getValue(phiB), targetLine.getValue(phiB) };

			for(std::size_t i = 0; i < 4; i++)
			{
				std::size_t gi = i / 2;
				areas.emplace_back(PointCloudType::Point({ coses[gi] * values[i], sins[gi] * values[i], 1 }));
			}
			*/
		}
	});

	return areas;
}

SphericalPlanesRegistration::PointCloudType::Location SphericalPlanesRegistration::gradientErrorAtZero(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes) const
{
	PointCloudType::Location gradient = PointCloudType::Location::Zero();
	iterPlanes(targetPlanes, sourcePlanes, [&gradient] (std::size_t, const geom::LidarPlane3& targetPlane, const geom::LidarPlane3& sourcePlane, const geom::Vector2d& topLeft, const geom::Vector2d& bottomRight)
	{
		gradient += targetPlane.gradientErrorAtZero(sourcePlane, topLeft, bottomRight);
	});

	return gradient;
}

void SphericalPlanesRegistration::alignSteps(std::size_t steps)
{
	align(targetPlanes, sourcePlanes, steps);
}

void SphericalPlanesRegistration::alignSteps(const PointCloudType::Transformation& guess, std::size_t steps)
{
	std::vector<geom::LidarPlane3> sourcePlanesT = sourcePlanes;
	geom::LidarPlane3::transformAll(sourcePlanesT, guess, true);
	removeInvisible(sourcePlanesT);

	align(targetPlanes, sourcePlanesT, steps);
	finalTransformation = finalTransformation * guess;
}

void SphericalPlanesRegistration::align()
{
	align(targetPlanes, sourcePlanes, maxIterations);
}

void SphericalPlanesRegistration::align(const PointCloudType::Transformation& guess)
{
	alignSteps(guess, maxIterations);
}

void SphericalPlanesRegistration::align(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, std::size_t maxIterations)
{
	if(targetPlanes.size() < 3 || sourcePlanes.size() < 3) return;

	PointCloudType::Location newInput;
	PointCloudType::Location input = PointCloudType::Location::Zero();

	std::vector<geom::LidarPlane3> sourcePlanesT = sourcePlanes;
	double tmpFinalError = error(targetPlanes, sourcePlanesT);

	evaluationCount = 1;
	converged = false;

	std::size_t i;
	for(i = 0; i < maxIterations; i++)
	{
		PointCloudType::Location gradient = gradientErrorAtZero(targetPlanes, sourcePlanesT);
		double newFinalError, newSumOfErrors;
		double globalGamma = 1;

		size_t tries = 0;
		do
		{
			for(std::size_t j = 0; j < 6; j++) newInput[j] = input[j] - globalGamma * gammas[j] * gradient[j];
			globalGamma *= 0.5;

			sourcePlanesT = sourcePlanes;
			geom::LidarPlane3::transformAllToLocation(sourcePlanesT, newInput, true);
			removeInvisible(sourcePlanesT);
			newFinalError = error(targetPlanes, sourcePlanesT);

			newSumOfErrors = sumOfErrors;

			evaluationCount++;

			tries++;
			if(tries > maxTries)
			{
				converged = true;
				goto mainLoopEnd;
			}
		}
		while(newFinalError > tmpFinalError);

		input = newInput;

		tmpFinalError = newFinalError;
		sumOfErrors = newSumOfErrors;

		if(tmpFinalError < minFinalError)
		{
			converged = true;
			break;
		}
	}
	mainLoopEnd:

	iterationCount = i;
	finalError = tmpFinalError;

	finalTransformation = geom::Transformable<PointCloudType::ScalarType, PointCloudType::dimension>::createTransformation(input);
}

void SphericalPlanesRegistration::testPrint() const
{
	iterPlanes(targetPlanes, sourcePlanes, [](std::size_t i, const geom::LidarPlane3& targetPlane, const geom::LidarPlane3& sourcePlane, const geom::Vector2d& beginBound, const geom::Vector2d& endBound)
	{
		std::cout << targetPlane.id << 'x' << sourcePlane.id << ": [" << beginBound.transpose() << "] x [" << endBound.transpose() << ']' << std::endl;
	}, true);
}

SphericalPlanesRegistration::Bound::Bound(double value, std::size_t scanId, bool beginBound, std::size_t index) :
	value(value),
	scanId(scanId),
	beginBound(beginBound),
	index(index)
{
}

bool SphericalPlanesRegistration::Bound::operator<(const SphericalPlanesRegistration::Bound& other) const
{
	if(value == other.value)
		return (!beginBound && other.beginBound);
	else
		return (value < other.value);
}

}}
