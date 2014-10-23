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

#include "lsl/probability/normaldistribution.hpp"

#include <cmath>
#include "lsl/utils/mathutils.hpp"

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace probability {

NormalDistribution::NormalDistribution(double mean, double stdDev)
{
	this->mean = mean;
	this->stdDev = stdDev;
}

double NormalDistribution::operator()(double x) const
{
	return 1 / (stdDev * MathUtils::SQRT_TWO_PI) * exp(- pow(x - mean, 2) / (2 * getVariance()));
}

}}
