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

#ifndef LSL_REGISTRATION_SPHERICALPLANESREGISTRATION_HPP
#define LSL_REGISTRATION_SPHERICALPLANESREGISTRATION_HPP

#include <functional>

#include "registration.hpp"

#include "lsl/containers/pointcloud.hpp"
#include "lsl/geom/lidarplane3.hpp"

#include "lsl/detection/ransac.hpp"

namespace lsl {
namespace registration {

class SphericalPlanesRegistration : public Registration<containers::PointCloud3d>
{
public:
	typedef std::function<void (std::size_t, const geom::LidarPlane3&, const geom::LidarPlane3&, const geom::Vector2d&, const geom::Vector2d&)> iterPlanesFunc;

private:
	class Bound
	{
	public:
		double value;
		std::size_t scanId;
		bool beginBound;
		std::size_t index;

		Bound() = default;
		Bound(double value, std::size_t scanId, bool beginBound, std::size_t index = -1);

		bool operator<(const Bound& other) const;
	};

	typedef std::function<void (const std::vector<const Bound*>& targetBounds, const std::vector<const Bound*>& sourceBounds)> iterBoundsFunc;

	detection::Ransac ransac;
	std::size_t maxPlaneCount = 10;

	// double maxDiffL;
	// double maxAvgDiffL;

	std::size_t maxIterations = 100;
	std::size_t maxTries = 20;
	double minFinalError = 1;
	std::vector<double> gammas = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

	double finalError = 0;

	double sumOfErrors = 0;
	// double coverFactor;
	// double coverAngleFactor;

	PointCloudType::Transformation finalTransformation = PointCloudType::Transformation::Identity();
	std::vector<geom::LidarPlane3> targetPlanes, sourcePlanes;

	PointCloudType::Transformation detectionTransformation = PointCloudType::Transformation::Identity();

	void iterBounds(const std::multiset<Bound>& bounds, iterBoundsFunc f) const;

	// double getAvgDiffL(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const;
	std::size_t iterAllPlanes(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, iterPlanesFunc f) const;
	std::size_t iterPlanes(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, iterPlanesFunc f, bool onlyCorresponding = true) const;

	void align(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes, std::size_t maxIterations);

public:
	virtual ~SphericalPlanesRegistration() = default;

	void loadConfig(const std::string& path);

	inline double getFinalError() const { return finalError; }
	inline PointCloudType::Transformation getFinalTransformation() const { return finalTransformation; }

	void setTarget(const PointCloudType& target);
	void setSource(const PointCloudType& source, bool setOldSourceAsTarget = false);

	inline const std::vector<geom::LidarPlane3>& getTargetPlanes() const { return targetPlanes; }
	void setTargetPlanes(const std::vector<geom::LidarPlane3>& targetPlanes);

	inline const std::vector<geom::LidarPlane3>& getSourcePlanes() const { return sourcePlanes; }
	void setSourcePlanes(const std::vector<geom::LidarPlane3>& sourcePlanes, bool setOldSourceAsTarget = false);

	inline detection::Ransac& getRansac() { return ransac; }

	// inline std::size_t getMaxLineCount() const { return maxLineCount; }
	// inline void setMaxLineCount(std::size_t maxLineCount) { this->maxLineCount = maxLineCount; }

	// inline double getMaxDiffL() const { return maxDiffL; }
	// inline void setMaxDiffL(double maxDiffL) { this->maxDiffL = maxDiffL; }

	// inline double getMaxAvgDiffL() const { return maxAvgDiffL; }
	// inline void setMaxAvgDiffL(double maxAvgDiffL) { this->maxAvgDiffL = maxAvgDiffL; }

	// inline std::size_t getMaxIterations() const { return maxIterations; }
	// inline void setMaxIterations(std::size_t maxIterations) { this->maxIterations = maxIterations; }

	// inline std::size_t getMaxTries() const { return maxTries; }
	// inline void setMaxTries(std::size_t maxTries) { this->maxTries = maxTries; }

	// inline double getMinFinalError() const { return minFinalError; }
	// inline void setMinFinalError(double minFinalError) { this->minFinalError = minFinalError; }

	// inline std::vector<double>& getGammas() { return gammas; }

	// inline double getSumOfErrors() const { return sumOfErrors; }
	// inline double getCoverFactor() const { return coverFactor; }
	// inline double getCoverAngleFactor() const { return coverAngleFactor; }

	inline void setDetectionTransformation(const PointCloudType::Transformation& transformation) { this->detectionTransformation = transformation; }
	inline const PointCloudType::Transformation& getDetectionTransformation() const { return detectionTransformation; }

	std::vector<geom::LidarPlane3> detectPlanes(const PointCloudType& points, std::size_t maxPlaneCount = -1) const;
	void removeInvisible(std::vector<geom::LidarPlane3>& planes) const;

	double error(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes);
	double errorTransform(const std::vector<geom::LidarPlane3>& targetPlanes, std::vector<geom::LidarPlane3> sourcePlanes, PointCloudType::Transformation transformation);

	PointCloudType errorAreas(const std::vector<geom::LidarPlane3>& targetPlanes, std::vector<geom::LidarPlane3> sourcePlanes, PointCloudType::Transformation transformation) const;

	PointCloudType::Location gradientErrorAtZero(const std::vector<geom::LidarPlane3>& targetPlanes, const std::vector<geom::LidarPlane3>& sourcePlanes) const;

	void alignSteps(std::size_t steps = 1);
	void alignSteps(const PointCloudType::Transformation& guess, std::size_t steps = 1);

	virtual void align();
	virtual void align(const PointCloudType::Transformation& guess);

	void testPrint() const;
};

}}

#endif // LSL_REGISTRATION_SPHERICALPLANESREGISTRATION_HPP
