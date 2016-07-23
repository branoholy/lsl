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

#ifndef LSL_GEOM_ORTHOGONALPOLYGONDECOMPOSITION_HPP
#define LSL_GEOM_ORTHOGONALPOLYGONDECOMPOSITION_HPP

#include "matrix.hpp"

namespace lsl {
namespace geom {

class OrthogonalPolygonDecomposition
{
private:
	Eigen::MatrixXi neighboursCount(const Eigen::MatrixXi& polygonEdges) const;
	Eigen::Vector2i* findLeader(const Eigen::MatrixXi& neighbours, int threshold) const;

public:
	Eigen::MatrixXi decompose(const Eigen::MatrixXi& polygonEdges) const;
};

}}

#endif // LSL_GEOM_ORTHOGONALPOLYGONDECOMPOSITION_HPP
