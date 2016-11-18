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

#include "lsl/registration/improvedllr.hpp"

namespace lsl {
namespace registration {

geom::Vector2d ImprovedLLR::minTranslation(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const
{
	geom::Vector2d gradient = geom::Vector2d::Zero();
	geom::Vector2d gradientNorm = geom::Vector2d::Zero();

	iterLines(targetLines, sourceLines, [&gradient, &gradientNorm] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		gradient += targetLine.gradientTranslationAtZero(sourceLine, phiA, phiB);
		gradientNorm += targetLine.gradientTranslationNormAtZero(sourceLine, phiA, phiB);
	});

	for(std::size_t i = 0; i < 2; i++)
	{
		gradient[i] /= gradientNorm[i];
	}

	return gradient;
}

double ImprovedLLR::gradientRotationAtZero(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const
{
	double gradientPhi = 0;

	iterLines(targetLines, sourceLines, [&gradientPhi] (std::size_t, const geom::LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		gradientPhi += targetLine.gradientRotationAtZero(sourceLine, phiA, phiB);
	});

	return gradientPhi;
}

void ImprovedLLR::align(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, std::size_t maxIterations)
{
	if(targetLines.size() < 2 || sourceLines.size() < 2) return;

	std::vector<geom::LidarLine2> sourceLinesT = sourceLines;
	auto computeError = [this, &targetLines, &sourceLines, &sourceLinesT](const PointCloudType::Location& input)
	{
		sourceLinesT = sourceLines;
		geom::LidarLine2::transformAllToLocation(sourceLinesT, input, true);
		removeInvisible(sourceLinesT);

		double errorValue = error(targetLines, sourceLinesT);
		evaluationCount++;

		return errorValue;
	};

	PointCloudType::Location input = PointCloudType::Location::Zero();
	double errorValue = error(targetLines, sourceLines);

	evaluationCount = 1;
	converged = false;

	bool canIncreaseGamma = true;
	double gamma = gammas[2];

	std::size_t i;
	for(i = 0; i < maxIterations; i++)
	{
		geom::Vector2d translation = minTranslation(targetLines, sourceLinesT);
		double gradient = gradientRotationAtZero(targetLines, sourceLinesT);

		PointCloudType::Location newInput;
		newInput[0] = input[0] - translation[0];
		newInput[1] = input[1] - translation[1];

		double gammaMul = 1;
		double newErrorValue = errorValue;

		while(true)
		{
			std::vector<geom::LidarLine2> tmpSourceLinesT = sourceLinesT;

			newInput[2] = input[2] - gamma * gradient;
			double tmpErrorValue = computeError(newInput);

			if(tmpErrorValue < newErrorValue)
			{
				newErrorValue = tmpErrorValue;

				if(gammaMul == 1)
				{
					if(canIncreaseGamma)
						gammaMul = 2;
					else
						break;
				}
				else if(gammaMul < 1)
					break;
			}
			else
			{
				if(gammaMul == 1)
				{
					gammaMul = 0.5;
					canIncreaseGamma = false;
				}
				else if(gammaMul > 1)
				{
					gamma *= 0.5;
					newInput[2] = input[2] - gamma * gradient;
					sourceLinesT = tmpSourceLinesT;
					break;
				}
			}

			gamma *= gammaMul;
		}

		bool finished = false;
		if(newErrorValue < errorValue)
		{
			finished = ((errorValue - newErrorValue) < 0.01);

			input = newInput;
			errorValue = newErrorValue;
		}

		if(errorValue < minFinalError || finished)
		{
			converged = true;
			break;
		}
	}

	iterationCount = i + 1;
	finalError = errorValue;

	finalTransformation = geom::Transformable<PointCloudType::ScalarType, PointCloudType::dimension>::createTransformation(input);
}

}}
