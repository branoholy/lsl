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

LLT::LLT()
{
}

void LLT::transform(vector<LidarLine2>& lines, double phi, double tx, double ty)
{
	double c = cos(phi);
	double s = sin(phi);

	vector<LidarLine2> linesToAdd;
	for(LidarLine2& line : lines)
	{
		line.transform(phi, c, s, tx, ty);

		// Divide lines crossing x+ axis (could happened only after transformation)
		double phiL = line.getPhiLow();
		double phiH = line.getPhiHigh();

		if(phiL > 0 && phiL < MathUtils::PI__TWO && phiH > MathUtils::THREE_PI__TWO && phiH < MathUtils::TWO_PI)
		{
			LidarLine2 lineDown = line;

			if(phiL == line.getPhiA())
			{
				line.setPhiB(0);
				lineDown.setPhiA(MathUtils::TWO_PI_EXCLUSIVE);
			}
			else
			{
				line.setPhiA(0);
				lineDown.setPhiB(MathUtils::TWO_PI_EXCLUSIVE);
			}

			linesToAdd.push_back(lineDown);
		}
	}

	lines.insert(lines.end(), linesToAdd.begin(), linesToAdd.end());
}

void LLT::removeInvisible(vector<LidarLine2>& lines)
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

double LLT::alignWithSOMA(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty) const
{
	Ransac ransac(100, 2, 40, 5);

	vector<LidarLine2> targetLines = ransac.run<LidarLine2>(target, 10);
	vector<LidarLine2> sourceLines = ransac.run<LidarLine2>(source, 10);

	phi = 0;
	tx = 0;
	ty = 0;

	return -1;
}

}}
