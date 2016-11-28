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

#include "lsl/registration/llr.hpp"

#include <set>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "lsl/geom/lidarline2.hpp"

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace registration {

LLR::LLR() : Registration(),
	maxLineCount(10),
	maxDiffL(std::numeric_limits<double>::max()),
	maxAvgDiffL(std::numeric_limits<double>::max()),
	maxIterations(50),
	maxTries(20),
	minFinalError(0.0001),
	gammas({ 0.1, 0.1, 0.00001 }),
	finalError(0),
	finalTransformation(PointCloudType::Transformation::Identity()),
	detectionTransformation(PointCloudType::Transformation::Identity())
{
}

void LLR::loadConfig(const std::string& path)
{
	auto config = YAML::LoadFile(path);
	auto yamlLineDetection = config["lineDetection"];
	auto yamlCorrespondences = config["correspondences"];
	auto yamlMinimization = config["minimization"];

	if(yamlLineDetection)
	{
		if(yamlLineDetection["maxLineCount"]) maxLineCount = yamlLineDetection["maxLineCount"].as<size_t>();

		auto yamlSplitMerge = yamlLineDetection["splitMerge"];
		if(yamlSplitMerge && yamlSplitMerge["minModelSize"] && yamlSplitMerge["maxError"])
		{
			splitMerge.set(yamlSplitMerge["minModelSize"].as<int>(), yamlSplitMerge["maxError"].as<double>());
		}

		auto yamlRansac = yamlLineDetection["ransac"];
		if(yamlRansac && yamlRansac["iterations"] && yamlRansac["minModelSize"] && yamlRansac["maxError"])
		{
			ransac.set(yamlRansac["iterations"].as<int>(), 2, yamlRansac["minModelSize"].as<int>(), yamlRansac["maxError"].as<double>());
		}
	}

	if(yamlCorrespondences)
	{
		if(yamlCorrespondences["maxDiffL"]) maxDiffL = yamlCorrespondences["maxDiffL"].as<double>();
		if(yamlCorrespondences["maxAvgDiffL"]) maxAvgDiffL = yamlCorrespondences["maxAvgDiffL"].as<double>();
	}

	if(yamlMinimization)
	{
		if(yamlMinimization["maxIterations"]) maxIterations = yamlMinimization["maxIterations"].as<std::size_t>();
		if(yamlMinimization["maxTries"]) maxTries = yamlMinimization["maxTries"].as<std::size_t>();
		if(yamlMinimization["minFinalError"]) minFinalError = yamlMinimization["minFinalError"].as<double>();
		if(yamlMinimization["minErrorDiff"]) minErrorDiff = yamlMinimization["minErrorDiff"].as<double>();
		if(yamlMinimization["gammas"]) gammas = yamlMinimization["gammas"].as<std::vector<double>>();
	}
}

void LLR::setTarget(const PointCloudType& target)
{
	targetLines = detectLines(target, maxLineCount);
}

void LLR::setSource(const PointCloudType& source, bool setOldSourceAsTarget)
{
	if(setOldSourceAsTarget) targetLines = std::move(sourceLines);
	sourceLines = detectLines(source, maxLineCount);
}

void LLR::setTargetLines(const std::vector<geom::LidarLine2>& targetLines)
{
	this->targetLines = targetLines;
}

void LLR::setSourceLines(const std::vector<geom::LidarLine2>& sourceLines, bool setOldSourceAsTarget)
{
	if(setOldSourceAsTarget) targetLines = std::move(this->sourceLines);
	this->sourceLines = sourceLines;
}

std::vector<geom::LidarLine2> LLR::detectLines(const PointCloudType& points, std::size_t maxLineCount) const
{
	if(maxLineCount == std::size_t(-1)) maxLineCount = this->maxLineCount;

	PointCloudType remainingPoints;
	std::vector<geom::LidarLine2> lines;

	if(detectionTransformation.isIdentity()) lines = splitMerge.run<geom::LidarLine2>(points, remainingPoints);
	else
	{
		PointCloudType pointsT = points;
		pointsT.transform(detectionTransformation);

		lines = splitMerge.run<geom::LidarLine2>(pointsT, remainingPoints);
	}

	if(lines.size() < maxLineCount)
	{
		std::vector<geom::LidarLine2> linesRansac = ransac.run<geom::LidarLine2>(remainingPoints, maxLineCount - lines.size());
		lines.insert(lines.end(), linesRansac.begin(), linesRansac.end());
	}

	if(!detectionTransformation.isIdentity())
	{
		lsl::geom::LidarLine2::transformAll(lines, detectionTransformation.transpose(), true);
	}

	removeInvisible(lines);

	return lines;
}

void LLR::removeInvisible(std::vector<geom::LidarLine2>& lines) const
{
	if(lines.empty()) return;

	std::set<IntervalEndpoint> intervals;
	for(geom::LidarLine2& line : lines)
	{
		const auto& intervalEndA = *intervals.insert(IntervalEndpoint(line.getPhiA())).first;
		const auto& intervalEndB = *intervals.insert(IntervalEndpoint(line.getPhiB())).first;

		line.intervalEndIndexA = &intervalEndA.index;
		line.intervalEndIndexB = &intervalEndB.index;
	}

	double *middleValues = new double[intervals.size() - 1];

	std::size_t intervalEndIndex = 0;
	auto itEnd = intervals.end();
	for(auto it = intervals.begin(); it != itEnd;)
	{
		const IntervalEndpoint& intervalEndA = *it++;
		intervalEndA.index = intervalEndIndex;

		if(it != itEnd)
		{
			const IntervalEndpoint& intervalEndB = *it;
			middleValues[intervalEndIndex] = (intervalEndA.value + intervalEndB.value) * 0.5;
		}
		intervalEndIndex++;
	}

	geom::LidarLine2 **tmpLines = utils::ArrayUtils::createFilledArray<geom::LidarLine2*>(intervals.size() - 1, nullptr);
	for(geom::LidarLine2& line : lines)
	{
		std::size_t startInterval = std::min(*line.intervalEndIndexA, *line.intervalEndIndexB);
		std::size_t endInterval = std::max(*line.intervalEndIndexA, *line.intervalEndIndexB);

		for(std::size_t i = startInterval; i < endInterval; i++)
		{
			if(tmpLines[i] == nullptr || tmpLines[i]->getValue(middleValues[i]) > line.getValue(middleValues[i]))
			{
				tmpLines[i] = &line;
			}
		}
	}

	delete[] middleValues;

	std::vector<geom::LidarLine2> visibleLines;
	visibleLines.reserve(intervals.size());

	std::size_t i = 0;
	geom::LidarLine2 *lastLine = nullptr;
	itEnd = std::prev(intervals.end());
	for(auto it = intervals.begin(); it != itEnd; i++)
	{
		IntervalEndpoint intervalA = *it++;
		IntervalEndpoint intervalB = *it;

		geom::LidarLine2 *line = tmpLines[i];
		if(line != nullptr && line->isVisible())
		{
			if(line == lastLine)
			{
				visibleLines.back().setPhiB(intervalB.value);
				intervals.erase(std::prev(it));
				itEnd = std::prev(intervals.end());
			}
			else
			{
				line->setPhiAB(intervalA.value, intervalB.value);
				visibleLines.push_back(*line);
				lastLine = line;
			}
		}
		else lastLine = nullptr;
	}

	lines = std::move(visibleLines);

	delete[] tmpLines;
}

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

std::size_t LLR::iterAllLines(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, iterFunc f) const
{
	return iterLines(targetLines, sourceLines, f, false);
}

std::size_t LLR::iterLines(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, LLR::iterFunc f, bool onlyCorresponding) const
{
	std::size_t size1 = targetLines.size(), size2 = sourceLines.size();
	std::size_t i1 = 0, i2 = 0;

	double avgDiffL = 0;
	if(onlyCorresponding && maxAvgDiffL != std::numeric_limits<double>::max()) avgDiffL = getAvgDiffL(targetLines, sourceLines);

	std::size_t counter = 0;
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

	return counter;
}

double LLR::error(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines)
{
	sumOfErrors = 0;
	coverAngleFactor = 0;

	iterLines(targetLines, sourceLines, [this] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		double ei = targetLine.error(sourceLine, phiA, phiB);

		if(ei >= 0) // TODO: Is it needed?
		{
			sumOfErrors += ei;
			coverAngleFactor += phiB - phiA;
		}
		else
		{
			std::cerr << "ei < 0: " << phiA << '-' << phiB << ": " << targetLine << ' ' << sourceLine << std::endl;
		}
	});

	finalError = coverFactor = std::numeric_limits<double>::max();
	if(coverAngleFactor > 0)
	{
		double one__caf = 1 / coverAngleFactor;
		coverFactor = one__caf * utils::MathUtils::TWO_PI;
		finalError = coverFactor * one__caf * sumOfErrors;
	}

	return finalError;
}

double LLR::errorTransform(const std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2> sourceLines, PointCloudType::Transformation transformation)
{
	geom::LidarLine2::transformAll(sourceLines, transformation, true);
	removeInvisible(sourceLines);

	return error(targetLines, sourceLines);
}

LLR::PointCloudType LLR::errorAreas(const std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2> sourceLines, PointCloudType::Transformation transformation) const
{
	geom::LidarLine2::transformAll(sourceLines, transformation, true);
	removeInvisible(sourceLines);

	PointCloudType areas;
	iterLines(targetLines, sourceLines, [&areas] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		double ei = targetLine.error(sourceLine, phiA, phiB);
		if(ei >= 0) // TODO: Is it needed?
		{
			double coses[] = { std::cos(phiA), std::cos(phiB) };
			double sins[] = { std::sin(phiA), std::sin(phiB) };
			double values[] = { targetLine.getValue(phiA), sourceLine.getValue(phiA), sourceLine.getValue(phiB), targetLine.getValue(phiB) };

			for(std::size_t i = 0; i < 4; i++)
			{
				std::size_t gi = i / 2;
				areas.emplace_back(PointCloudType::Point({ coses[gi] * values[i], sins[gi] * values[i], 1 }));
			}
		}
	});

	return areas;
}

LLR::PointCloudType::Location LLR::gradientErrorAtZero(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const
{
	PointCloudType::Location gradient = PointCloudType::Location::Zero();
	iterLines(targetLines, sourceLines, [&gradient] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		gradient += targetLine.gradientErrorAtZero(sourceLine, phiA, phiB);
	});

	return gradient;
}

void LLR::alignSteps(std::size_t steps)
{
	align(targetLines, sourceLines, steps);
}

void LLR::alignSteps(const PointCloudType::Transformation& guess, std::size_t steps)
{
	std::vector<geom::LidarLine2> sourceLinesT = sourceLines;
	geom::LidarLine2::transformAll(sourceLinesT, guess, true);
	removeInvisible(sourceLinesT);

	align(targetLines, sourceLinesT, steps);
	finalTransformation = finalTransformation * guess;
}

void LLR::align()
{
	align(targetLines, sourceLines, maxIterations);
}

void LLR::align(const PointCloudType::Transformation& guess)
{
	alignSteps(guess, maxIterations);
}

void LLR::align(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, std::size_t maxIterations)
{
	if(targetLines.size() < 2 || sourceLines.size() < 2) return;

	PointCloudType::Location newInput;
	PointCloudType::Location input = { 0, 0, 0 };

	std::vector<geom::LidarLine2> sourceLinesT = sourceLines;
	double tmpFinalError = error(targetLines, sourceLinesT);

	evaluationCount = 1;
	converged = false;

	std::size_t i;
	for(i = 0; i < maxIterations; i++)
	{
		PointCloudType::Location gradient = gradientErrorAtZero(targetLines, sourceLinesT);

		double newFinalError, newSumOfErrors, newCoverFactor, newCoverAngleFactor;
		double globalGamma = 1;

		size_t tries = 0;
		do
		{
			for(std::size_t j = 0; j < 3; j++) newInput[j] = input[j] - globalGamma * gammas[j] * gradient[j];
			globalGamma *= 0.5;

			sourceLinesT = sourceLines;
			geom::LidarLine2::transformAllToLocation(sourceLinesT, newInput, true);
			removeInvisible(sourceLinesT);
			newFinalError = error(targetLines, sourceLinesT);

			newSumOfErrors = sumOfErrors;
			newCoverFactor = coverFactor;
			newCoverAngleFactor = coverAngleFactor;

			evaluationCount++;

			tries++;
			if(tries > maxTries)
			{
				converged = true;
				goto mainLoopEnd;
			}
		}
		while(newFinalError > tmpFinalError);

		bool finished = false;
		if(newFinalError < tmpFinalError)
		{
			finished = ((tmpFinalError - newFinalError) < minErrorDiff);

			input = newInput;

			tmpFinalError = newFinalError;
			sumOfErrors = newSumOfErrors;
			coverFactor = newCoverFactor;
			coverAngleFactor = newCoverAngleFactor;
		}

		if(tmpFinalError < minFinalError || finished)
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

LLR::IntervalEndpoint::IntervalEndpoint(double value) :
	value(value)
{
}

bool LLR::IntervalEndpoint::operator<(const LLR::IntervalEndpoint& other) const
{
	return (value < other.value);
}

}}
