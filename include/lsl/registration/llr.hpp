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

#ifndef LSL_REGISTRATION_LLR_HPP
#define LSL_REGISTRATION_LLR_HPP

#include <functional>

#include "lsl/containers/pointcloud.hpp"

#include "lsl/geom/lidarline2.hpp"
#include "lsl/geom/ransac.hpp"
#include "lsl/geom/splitmerge.hpp"

#include "registration.hpp"

namespace lsl {
namespace registration {

class LLR : public Registration<containers::PointCloud2d>
{
public:
	typedef std::function<void(std::size_t, const geom::LidarLine2&, const geom::LidarLine2&, double, double)> iterFunc;

private:
	geom::Ransac ransac;
	geom::SplitMerge splitMerge;
	std::size_t maxLineCount;

	double maxDiffL;
	double maxAvgDiffL;

	std::size_t maxIterations;
	std::size_t maxTries;
	double minFinalError;
	std::vector<double> gammas;

	double finalError;

	double sumOfErrors;
	double coverFactor;
	double coverAngleFactor;

	PointCloudType::Transformation finalTransformation;
	std::vector<geom::LidarLine2> targetLines, sourceLines;

	double getAvgDiffL(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const;
	std::size_t iterAllLines(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, iterFunc f) const;
	std::size_t iterLines(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, iterFunc f, bool onlyCorresponding = true) const;

	void align(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, std::size_t maxIterations);

	class IntervalEndpoint
	{
	public:
		double value;
		mutable std::size_t index;

		IntervalEndpoint() = default;
		IntervalEndpoint(double value);

		bool operator<(const IntervalEndpoint& other) const;
	};

public:
	LLR();

	void loadConfig(const std::string& path);

	inline double getFinalError() const { return finalError; }
	inline PointCloudType::Transformation getFinalTransformation() const { return finalTransformation; }
	PointCloudType::Location getFinalLocation() const;

	void setTarget(const PointCloudType& target);
	void setSource(const PointCloudType& source, bool setOldSourceAsTarget = false);

	inline const std::vector<geom::LidarLine2>& getTargetLines() const { return targetLines; }
	void setTargetLines(const std::vector<geom::LidarLine2>& targetLines);

	inline const std::vector<geom::LidarLine2>& getSourceLines() const { return sourceLines; }
	void setSourceLines(const std::vector<geom::LidarLine2>& sourceLines, bool setOldSourceAsTarget = false);

	inline geom::Ransac& getRansac() { return ransac; }
	inline geom::SplitMerge& getSplitMerge() { return splitMerge; }

	inline std::size_t getMaxLineCount() const { return maxLineCount; }
	inline void setMaxLineCount(std::size_t maxLineCount) { this->maxLineCount = maxLineCount; }

	inline double getMaxDiffL() const { return maxDiffL; }
	inline void setMaxDiffL(double maxDiffL) { this->maxDiffL = maxDiffL; }

	inline double getMaxAvgDiffL() const { return maxAvgDiffL; }
	inline void setMaxAvgDiffL(double maxAvgDiffL) { this->maxAvgDiffL = maxAvgDiffL; }

	inline std::size_t getMaxIterations() const { return maxIterations; }
	inline void setMaxIterations(std::size_t maxIterations) { this->maxIterations = maxIterations; }

	inline std::size_t getMaxTries() const { return maxTries; }
	inline void setMaxTries(std::size_t maxTries) { this->maxTries = maxTries; }

	inline double getMinFinalError() const { return minFinalError; }
	inline void setMinFinalError(double minFinalError) { this->minFinalError = minFinalError; }

	inline std::vector<double>& getGammas() { return gammas; }

	inline double getSumOfErrors() const { return sumOfErrors; }
	inline double getCoverFactor() const { return coverFactor; }
	inline double getCoverAngleFactor() const { return coverAngleFactor; }

	std::vector<geom::LidarLine2> detectLines(const PointCloudType& points, std::size_t maxLineCount = -1) const;
	void removeInvisible(std::vector<geom::LidarLine2>& lines) const;

	double error(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines);
	double errorTransform(const std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2> sourceLines, PointCloudType::Transformation transformation);

	PointCloudType errorAreas(const std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2> sourceLines, PointCloudType::Transformation transformation) const;

	PointCloudType::Location gradientErrorAtZero(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const;

	void alignSteps(std::size_t steps = 1);
	void alignSteps(const PointCloudType::Transformation& guess, std::size_t steps = 1);

	virtual void align();
	virtual void align(const PointCloudType::Transformation& guess);
};

}}

#endif // LSL_REGISTRATION_LLR_HPP
