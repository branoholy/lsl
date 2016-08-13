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

#ifndef LSL_UTILS_TOSTRINGMIXIN_HPP
#define LSL_UTILS_TOSTRINGMIXIN_HPP

#include <string>

namespace lsl {
namespace utils {

class ToStringMixin
{
public:
	virtual std::ostream& toStream(std::ostream& out) const;
	virtual std::string toString() const final;

	friend std::ostream& operator<<(std::ostream& out, const ToStringMixin& mixin);
};

}}

#endif // LSL_UTILS_TOSTRINGMIXIN_HPP
