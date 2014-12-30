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

#ifndef LSL_REGISTRATION_LLT_HPP
#define LSL_REGISTRATION_LLT_HPP

#include <vector>
#include <functional>

#include "lsl/geom/vector.hpp"
#include "lsl/geom/lidarline2.hpp"

namespace lsl {
namespace registration {

class LLT
{
public:
	typedef std::function<void(std::size_t, const geom::LidarLine2&, const geom::LidarLine2&, double, double)> iterFunc;

private:
	std::size_t evals;
	bool converged;

	void iterLines(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, iterFunc f) const;

public:
	LLT();

	inline std::size_t getEvaluationCount() const { return evals; }
	inline bool hasConverged() const { return converged; }

	void detectLines(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2>& sourceLines) const;
	void removeInvisible(std::vector<geom::LidarLine2>& lines) const;
	double error(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines) const;

	double errorTransform(const std::vector<geom::LidarLine2>& targetLines, std::vector<geom::LidarLine2> sourceLines, double phi, double tx, double ty) const;

	double alignWithSOMA(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, double& phi, double& tx, double& ty, bool withGuess = false);
	double alignWithSOMA(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, double& phi, double& tx, double& ty, bool withGuess = false);

	double alignWithGD(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, double& phi, double& tx, double& ty);
	double alignWithGD(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, double& phi, double& tx, double& ty);

	double alignWithNewton(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, double& phi, double& tx, double& ty);
	double alignWithNewton(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, double& phi, double& tx, double& ty);

	double alignWithLeastSquare(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, double& phi, double& tx, double& ty);
	double alignWithLeastSquare(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, double& phi, double& tx, double& ty);

	double alignWithSOMALS(const std::vector<geom::Vector2d>& target, const std::vector<geom::Vector2d>& source, double& phi, double& tx, double& ty, bool withGuess = false);
	double alignWithSOMALS(const std::vector<geom::LidarLine2>& targetLines, const std::vector<geom::LidarLine2>& sourceLines, double& phi, double& tx, double& ty, bool withGuess = false);

};

}}

#endif // LSL_REGISTRATION_LLT_HPP
