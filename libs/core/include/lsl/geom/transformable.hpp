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

#ifndef LSL_GEOM_TRANSFORMABLE_HPP
#define LSL_GEOM_TRANSFORMABLE_HPP

#undef NDEBUG
#include <cassert>

#include "matrix.hpp"

namespace lsl {
namespace geom {

template<typename ScalarType, int dim>
class Transformable
{
public:
	typedef Matrix<ScalarType, 3 * dim - 3, 1> Location; // x, y, theta;;; x, y, z, roll, pitch, yaw ?
	typedef Matrix<ScalarType, dim + 1, dim + 1> Transformation; // counter-clockwise rotation

	virtual void transform(const Transformation& transformation) = 0;
	virtual void transformToLocation(const Location& location);

	static Transformation createTransformation(const Location& location);
	static Location createLocation(const Transformation& transformation);
};

template<typename ScalarType, int dim>
void Transformable<ScalarType, dim>::transformToLocation(const Location& location)
{
	return transform(createTransformation(location));
}

template<typename ScalarType, int dim>
typename Transformable<ScalarType, dim>::Transformation Transformable<ScalarType, dim>::createTransformation(const Location& location)
{
	// TODO: Implement for 3D as well.
	assert(dim == 2 && "Dimension should be 2.");

	double c = std::cos(location[2]);
	double s = std::sin(location[2]);

	Transformation transformation;
	transformation << c, -s, location[0],
					  s, c, location[1],
					  0, 0, 1;

	return transformation;
}

template<typename ScalarType, int dim>
typename Transformable<ScalarType, dim>::Location Transformable<ScalarType, dim>::createLocation(const Transformation& transformation)
{
	// TODO: Implement for 3D as well.
	assert(dim == 2 && "Dimension should be 2.");

	return {transformation(0, 2), transformation(1, 2), std::atan2(transformation(1, 0), transformation(0, 0))};
}

}}

#endif // LSL_GEOM_TRANSFORMABLE_HPP
