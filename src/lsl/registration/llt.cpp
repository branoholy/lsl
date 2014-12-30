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

#include <Eigen/Dense>
#include <Eigen/LU>

#include "lsl/geom/lidarline2.hpp"
#include "lsl/geom/ransac.hpp"

#include "lsl/optimization/gradientdescent.hpp"
#include "lsl/optimization/newton.hpp"
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

void LLT::detectLines(const vector<Vector2d>& target, const vector<Vector2d>& source, vector<LidarLine2>& targetLines, vector<LidarLine2>& sourceLines) const
{
	Ransac ransac(100, 2, 40, 5);

	targetLines = ransac.run<LidarLine2>(target, 10);
	sourceLines = ransac.run<LidarLine2>(source, 10);

	// TODO: Change to orderLines()
	removeInvisible(targetLines);
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

void LLT::iterLines(const vector<LidarLine2>& targetLines, const vector<LidarLine2>& sourceLines, LLT::iterFunc f) const
{
	size_t size1 = targetLines.size(), size2 = sourceLines.size();
	size_t i1 = 0, i2 = 0;

	size_t counter = 0;
	while(i1 < size1 && i2 < size2)
	{
		const LidarLine2& targetLine = targetLines.at(i1);
		const LidarLine2& sourceLine = sourceLines.at(i2);

		double maxPhiA = max(targetLine.getPhiA(), sourceLine.getPhiA());
		double minPhiB = min(targetLine.getPhiB(), sourceLine.getPhiB());

		if(maxPhiA < minPhiB)
		{
			f(counter++, targetLine, sourceLine, maxPhiA, minPhiB);
		}

		if(minPhiB >= targetLine.getPhiB()) i1++;
		if(minPhiB >= sourceLine.getPhiB()) i2++;
	}
}

double LLT::error(const vector<LidarLine2>& targetLines, const vector<LidarLine2>& sourceLines) const
{
	double error = 0;
	double intersectDfSum = 0;

	iterLines(targetLines, sourceLines, [&error, &intersectDfSum](size_t, const LidarLine2& targetLine, const geom::LidarLine2& sourceLine, double phiA, double phiB)
	{
		double ei = targetLine.error(sourceLine, phiA, phiB);
		if(ei >= 0)
		{
			error += ei;
			intersectDfSum += phiB - phiA;
		}
	});

	// double targetDfSum = LidarLine2::sumDf(targetLines);
	// double sourceDfSum = LidarLine2::sumDf(sourceLines);
	// double minDfSum = (targetDfSum + sourceDfSum) / 2;

	double finalError = numeric_limits<double>::max();
	if(intersectDfSum > 0) finalError = (MathUtils::TWO_PI / (intersectDfSum * intersectDfSum)) * error;

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

double LLT::alignWithSOMA(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty, bool withGuess)
{
	vector<LidarLine2> targetLines, sourceLines;
	detectLines(target, source, targetLines, sourceLines);

	return alignWithSOMA(targetLines, sourceLines, phi, tx, ty, withGuess);
}

double LLT::alignWithSOMA(const std::vector<LidarLine2>& targetLines, const std::vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty, bool withGuess)
{
	// SOMA soma(3, 3, 0.1, 0.5, 30, 100, 0.001);
	// SOMA soma(3, 2, 0.11, 0.8, 30, 100, 0.001);
	SOMA soma(3, 2, 0.31, 0.8, 50, 1000, 0.0001);

	const double specimen[][2] = {{phi - MathUtils::PI__TWO, phi + MathUtils::PI__TWO}, {tx - 300, tx + 300}, {ty - 200, ty + 200}};
	soma.setSpecimen(specimen);

	if(withGuess)
	{
		const double mean[] = {phi, tx, ty};
		const double sigmas[] = {2, 20, 20};
		soma.initNormalPopulation(mean, sigmas);
	}
	else
	{
		soma.initUniformPopulation();
	}

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

double LLT::alignWithGD(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty)
{
	vector<LidarLine2> targetLines, sourceLines;
	detectLines(target, source, targetLines, sourceLines);

	return alignWithGD(targetLines, sourceLines, phi, tx, ty);
}

double LLT::alignWithGD(const std::vector<LidarLine2>& targetLines, const std::vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty)
{
	double deltas[] = {0.0000001, 0.00001, 0.00001};
	double gammas[] = {0.000001, 0.00001, 0.00001};
	GradientDescent gd(3, deltas, gammas, 0.001);

	double error;
	const double guess[] = {phi, tx, ty};
	double *opt = gd.minimize([this, &targetLines, &sourceLines](const double *transform) -> double
	{
		return this->errorTransform(targetLines, sourceLines, transform[0], transform[1], transform[2]);
	}, guess, error);

	evals = gd.getEvaluationCount();
	converged = gd.hasConverged();

	phi = opt[0];
	tx = opt[1];
	ty = opt[2];

	delete[] opt;

	return error;
}

double LLT::alignWithNewton(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty)
{
	vector<LidarLine2> targetLines, sourceLines;
	detectLines(target, source, targetLines, sourceLines);

	return alignWithNewton(targetLines, sourceLines, phi, tx, ty);
}

double LLT::alignWithNewton(const vector<LidarLine2>& targetLines, const vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty)
{
	double deltas[] = {0.0000001, 0.00001, 0.00001};
	double gammas[] = {0.0001, 0.001, 0.001};
	Newton newton(3, deltas, gammas, 0.000001);

	double error;
	const double guess[] = {phi, tx, ty};
	double *opt = newton.minimize([this, &targetLines, &sourceLines](const double *transform) -> double
	{
		return this->errorTransform(targetLines, sourceLines, transform[0], transform[1], transform[2]);
	}, guess, error);

	evals = newton.getEvaluationCount();
	converged = newton.hasConverged();

	phi = opt[0];
	tx = opt[1];
	ty = opt[2];

	delete[] opt;

	return error;
}

double LLT::alignWithLeastSquare(const vector<Vector2d>& target, const vector<Vector2d>& source, double& phi, double& tx, double& ty)
{
	vector<LidarLine2> targetLines, sourceLines;
	detectLines(target, source, targetLines, sourceLines);

	return alignWithLeastSquare(targetLines, sourceLines, phi, tx, ty);
}

double LLT::alignWithLeastSquare(const vector<LidarLine2>& targetLines, const vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty)
{
	converged = false;
	evals = 0;

	double lastErrorT = numeric_limits<double>::max();
	double bestErrorT = numeric_limits<double>::max();
	Eigen::Matrix3d bestT = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d allT = Eigen::Matrix3d::Identity();

	vector<LidarLine2> tmpSourceLines = sourceLines;
	removeInvisible(tmpSourceLines);

	for(size_t i = 0; i < 100; i++)
	{
		Eigen::VectorXd vl(1);
		Eigen::MatrixX2d mc(1, 2);

		size_t iters = 0;
		double angleX = 0, angleY = 0;
		iterLines(targetLines, tmpSourceLines, [&iters, &angleX, &angleY, &vl, &mc](size_t i, const LidarLine2& lineA, const LidarLine2& lineB, double, double)
		{
			iters = i + 1;

			double angleDiff = lineA.getAlpha() - lineB.getAlpha();
			angleX += cos(angleDiff);
			angleY += sin(angleDiff);

			vl.conservativeResize(iters);
			mc.conservativeResize(iters, Eigen::NoChange);

			vl(i) = lineA.getL() - lineB.getL();
			mc(i, 0) = cos(lineA.getAlpha());
			mc(i, 1) = sin(lineA.getAlpha());
		});

		cout << vl << endl << endl << mc << endl << endl;

		angleX /= iters;
		angleY /= iters;
		double avgAngle = atan2(angleY, angleX);
		cout << avgAngle << endl;

		Eigen::MatrixXd mTmp = mc.transpose() * mc;
		Eigen::FullPivLU<Eigen::MatrixXd> luTmp(mTmp);

		if(luTmp.isInvertible())
		{
			Eigen::Vector2d vt = mTmp.inverse() * mc.transpose() * vl;
			cout << vt << endl;

			double c = cos(avgAngle);
			double s = sin(avgAngle);

			Eigen::Matrix3d tmpT;
			tmpT << c, -s, vt(0),
				 s, c, vt(1),
				 0, 0, 1;

			allT = tmpT * allT;

			// double tmpPhi = atan2(allT(0, 1), allT(0, 0));
			// double tmpTx = allT(0, 2);
			// double tmpTy = allT(1, 2);
			double tmpPhi = avgAngle;
			double tmpTx = vt(0);
			double tmpTy = vt(1);

			// tmpSourceLines = sourceLines;
			LidarLine2::transform(tmpSourceLines, tmpPhi, tmpTx, tmpTy);
			removeInvisible(tmpSourceLines);
			double errorT = error(targetLines, tmpSourceLines);
			evals++;

			cout << i << ". error: " << errorT << '/' << lastErrorT << ' ' << abs(errorT - lastErrorT) << endl;

			if(errorT < bestErrorT)
			{
				bestErrorT = errorT;
				bestT = allT;
			}

			if(abs(errorT - lastErrorT) < 0.000001)
			{
				converged = true;
				break;
			}

			lastErrorT = errorT;
		}
		else
		{
			cout << "Matrix is not invertible." << endl;
			converged = false;
			break;
		}

		cout << "-----------------------------------" << endl;
	}

	cout << "Best T:" << endl << bestT << endl;

	phi = atan2(bestT(1, 0), bestT(0, 0));
	tx = bestT(0, 2);
	ty = bestT(1, 2);

	return bestErrorT;
}

double LLT::alignWithSOMALS(const std::vector<Vector2d>& target, const std::vector<Vector2d>& source, double& phi, double& tx, double& ty, bool withGuess)
{
	vector<LidarLine2> targetLines, sourceLines;
	detectLines(target, source, targetLines, sourceLines);

	return alignWithSOMALS(targetLines, sourceLines, phi, tx, ty, withGuess);
}

double LLT::alignWithSOMALS(const std::vector<LidarLine2>& targetLines, const std::vector<LidarLine2>& sourceLines, double& phi, double& tx, double& ty, bool withGuess)
{
	double phiTip, txTip, tyTip;
	alignWithLeastSquare(targetLines, sourceLines, phiTip, txTip, tyTip);
	double lsEvals = evals;
	cout << "LS converged: " << converged << endl;

	double error;
	if(converged)
	{

		SOMA soma(3, 1.3, 0.31, 0.8, 20, 1000, 0.0001);

		const double specimen[][2] = {{phi - MathUtils::PI__TWO, phi + MathUtils::PI__TWO}, {tx - 300, tx + 300}, {ty - 200, ty + 200}};
		soma.setSpecimen(specimen);

		phi = phiTip;
		tx = txTip;
		ty = tyTip;

		const double mean[] = {phi, tx, ty};
		const double sigmas[] = {0.5, 5, 5};
		soma.initNormalPopulation(mean, sigmas);

		const double leaderTip[] = {phiTip, txTip, tyTip};
		double *opt = soma.minimize([this, &targetLines, &sourceLines](const double *transform) -> double
		{
			return this->errorTransform(targetLines, sourceLines, transform[0], transform[1], transform[2]);
		}
		, leaderTip, error);

		evals += soma.getEvaluationCount();
		converged = soma.hasConverged();

		phi = opt[0];
		tx = opt[1];
		ty = opt[2];

		delete[] opt;


		/*
		phi = phiTip;
		tx = txTip;
		ty = tyTip;

		error = alignWithGD(targetLines, sourceLines, phi, tx, ty);
		evals += lsEvals;
		*/
	}
	else
	{
		error = alignWithSOMA(targetLines, sourceLines, phi, tx, ty, withGuess);
		evals += lsEvals;
	}

	return error;
}

}}
