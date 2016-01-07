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

#ifndef LSL_REGISTRATION_ICP_HPP
#define LSL_REGISTRATION_ICP_HPP

#include "registration.hpp"

#include "lsl/containers/pointcloud.hpp"

namespace lsl {
namespace registration {

class ICP : public Registration<containers::PointCloud2d>
{
private:
	std::size_t maxIterations;
	double minFinalError;
	double maxDistance2;

	double finalError;
	PointCloudType::Transformation finalTransformation;
	PointCloudType target, source;

	void align(const PointCloudType& target, const PointCloudType& source, std::size_t maxIterations);

protected:
	virtual double matchPoints(const PointCloudType& target, const PointCloudType& source, std::vector<std::size_t>& matches);
	virtual PointCloudType::Transformation computeTransformation(const PointCloudType& target, const PointCloudType& source, const std::vector<std::size_t>& matches);

public:
	ICP();

	void loadConfig(const std::string& path);

	inline double getFinalError() const { return finalError; }
	inline PointCloudType::Transformation getFinalTransformation() const { return finalTransformation; }

	void setTarget(const PointCloudType& target);
	void setSource(const PointCloudType& source, bool setOldSourceAsTarget = false);

	inline std::size_t getMaxIterations() const { return maxIterations; }
	inline void setMaxIterations(std::size_t maxIterations) { this->maxIterations = maxIterations; }

	inline double getMinFinalError() const { return minFinalError; }
	inline void setMinFinalError(double minFinalError) { this->minFinalError = minFinalError; }

	void alignSteps(std::size_t steps = 1);
	void alignSteps(const PointCloudType::Transformation& guess, std::size_t steps = 1);

	virtual void align();
	virtual void align(const PointCloudType::Transformation& guess);
};

}}

#endif // LSL_REGISTRATION_ICP_HPP
