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

#ifndef LSL_REGISTRATION_REGISTRATION_HPP
#define LSL_REGISTRATION_REGISTRATION_HPP

#include "lsl/containers/pointcloud.hpp"

namespace lsl {
namespace registration {

template<typename PointCloudT>
class Registration
{
protected:
	std::size_t evaluationCount;
	std::size_t iterationCount;
	bool converged;

public:
	typedef PointCloudT PointCloudType;

	Registration();
	virtual ~Registration() {}

	virtual void loadConfig(const std::string& path);

	inline std::size_t getEvaluationCount() const { return evaluationCount; }
	inline std::size_t getIterationCount() const { return iterationCount; }
	inline bool hasConverged() const { return converged; }

	virtual double getFinalError() const = 0;
	virtual typename PointCloudT::Transformation getFinalTransformation() const = 0;

	virtual void setTarget(const PointCloudT& target) = 0;
	virtual void setSource(const PointCloudT& source, bool setOldSourceAsTarget = false) = 0;

	virtual void align() = 0;
	virtual void align(const typename PointCloudT::Transformation& guess) = 0;
};

template<typename PointCloudT>
Registration<PointCloudT>::Registration() :
	evaluationCount(0), iterationCount(0), converged(false)
{
}

template<typename PointCloudT>
void Registration<PointCloudT>::loadConfig(const std::string&)
{
}

}}

#endif // LSL_REGISTRATION_LLT_HPP
