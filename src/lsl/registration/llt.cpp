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

#include "lsl/registration/llt.hpp"

#include <set>

#include "lsl/geom/lidarline2.hpp"
#include "lsl/geom/ransac.hpp"
#include "lsl/optimization/soma.hpp"
#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/mathutils.hpp"

using namespace std;
using namespace lsl::geom;
using namespace lsl::optimization;
using namespace lsl::utils;

namespace lsl {
namespace registration {

LLT::LLT() :
	evals(0), converged(false)
{
}

void LLT::removeInvisible(vector<LidarLine2>& lines) const
{
	set<double> intervals;
	for(const LidarLine2& line : lines)
	{
		intervals.insert(line.getPhiA());
		intervals.insert(line.getPhiB());
	}

	vector<LidarLine2*> tmpLines;
	tmpLines.reserve(intervals.size());

	LidarLine2 *lastMinLine = nullptr;

	auto itEnd = prev(intervals.end());
	for(auto it = intervals.begin(); it != itEnd;)
	{
		double phiA = *it++;
		double phiB = *it;
		double phiC = (phiA + phiB) * 0.5;

		double minValue = numeric_limits<double>::max();
		LidarLine2 *minLine = nullptr;

		for(LidarLine2& line : lines)
		{
			double value = line.getValue(phiC);
			if(value < minValue)
			{
				minValue = value;
				minLine = &line;
			}
		}

		if(lastMinLine != nullptr && minLine == lastMinLine)
		{
			intervals.erase(prev(it));
			itEnd = prev(intervals.end());
		}
		else
		{
			tmpLines.push_back(minLine);
			lastMinLine = minLine;
		}
	}

	std::vector<LidarLine2> visibleLines;
	visibleLines.reserve(tmpLines.size());

	size_t i = 0;
	for(auto it = intervals.begin(); it != itEnd; i++)
	{
		double phiA = *it++;
		double phiB = *it;

		LidarLine2 *line = tmpLines.at(i);
		if(line != nullptr && line->isVisible())
		{
			line->setPhiA(phiA);
			line->setPhiB(phiB);
			visibleLines.push_back(*line);
		}
	}

	lines = move(visibleLines);
}

double LLT::error(const std::vector<LidarLine2>& targetLines, const std::vector<LidarLine2>& sourceLines) const
{
	size_t size1 = targetLines.size(), size2 = sourceLines.size();
	size_t i1 = 0, i2 = 0;
	double error = 0;
	double intervalSum = 0;

	size_t counter = 0;

	while(i1 < size1 && i2 < size2)
	{
		counter++;
		const LidarLine2& targetLine = targetLines.at(i1);
		const LidarLine2& sourceLine = sourceLines.at(i2);

		double maxPhiA = max(targetLine.getPhiA(), sourceLine.getPhiA());
		double minPhiB = min(targetLine.getPhiB(), sourceLine.getPhiB());

		if(maxPhiA < minPhiB)
		{
			double ei = targetLine.error(sourceLine, maxPhiA, minPhiB);
			if(ei >= 0)
			{
				error += ei;
				intervalSum += minPhiB - maxPhiA;
			}
			else
			{
				// cout << '<' << maxPhiA << "; " << minPhiB << "> = " << ei << endl;
				// cout << targetLine << endl << sourceLine << endl << endl;
			}
		}

		if(minPhiB >= targetLine.getPhiB()) i1++;
		if(minPhiB >= sourceLine.getPhiB()) i2++;
	}

	/*
	cout << "Counter: " << counter << endl;
	cout << "Target size: " << size1 << endl;
	cout << "Source size: " << size2 << endl;
	*/

	double finalError = numeric_limits<double>::max();
	if(intervalSum > 0) finalError = (MathUtils::TWO_PI / (intervalSum * intervalSum)) * error;

	// cout << finalError << ' ' << intervalSum << ' ' << error << endl;

	return finalError;
}

double LLT::errorTransform(const std::vector<LidarLine2>& targetLines, std::vector<LidarLine2> sourceLines, double phi, double tx, double ty) const
{
	LidarLine2::transform(sourceLines, phi, tx, ty);
	removeInvisible(sourceLines);
	double errorT = error(targetLines, sourceLines);

	// cout << '{' << phi << ", " << tx << ", " << ty << "} = " << errorT << endl;

	return errorT;
}

double LLT::alignWithSOMA(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty)
{
	Ransac ransac(100, 2, 40, 5);

	vector<LidarLine2> targetLines = ransac.run<LidarLine2>(target, 10);
	vector<LidarLine2> sourceLines = ransac.run<LidarLine2>(source, 10);

	// TODO: Change to orderLines()
	removeInvisible(targetLines);

	return alignWithSOMA(targetLines, sourceLines, phi, tx, ty);
}

double LLT::alignWithSOMA(const std::vector<LidarLine2>& targetLines, const std::vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty)
{
	// SOMA soma(3, 3, 0.1, 0.5, 30, 100, 0.001);
	// SOMA soma(3, 2, 0.11, 0.8, 30, 100, 0.001);
	SOMA soma(3, 2, 0.31, 0.8, 50, 1000, 0.00001);

	const double specimen[][2] = {{phi - MathUtils::PI__TWO, phi + MathUtils::PI__TWO}, {tx - 300, tx + 300}, {ty - 200, ty + 200}};
	soma.setSpecimen(specimen);

	soma.initUniformPopulation();
	// const double mean[] = {phi, tx, ty};
	// const double sigmas[] = {2, 20, 20};
	// soma.initNormalPopulation(mean, sigmas);

	double error;
	double *opt = soma.minimize([this, &targetLines, &sourceLines](const double *transform) -> double
	{
		return this->errorTransform(targetLines, sourceLines, transform[0], transform[1], transform[2]);
	}
	, error);

	evals = soma.getEvaluationCount();
	converged = soma.hasConverged();

	phi = opt[0];
	tx = opt[1];
	ty = opt[2];

	delete[] opt;

	return error;
}

}}
