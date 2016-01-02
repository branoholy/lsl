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

protected:
	int id;

public:
	inline Scalar at(uint i, uint j) const { return this->operator()(i,j); }
	inline Scalar& at(uint i, uint j) { return this->operator()(i,j); }
	inline Scalar at(uint i) const { return this->operator[](i); }
	inline Scalar& at(uint i) { return this->operator[](i); }

	inline int getId() const { return id; }
	inline void setId(int id) { this->id = id; }

	inline double getAngle2D() const { return std::atan2(at(1), at(0)); }
