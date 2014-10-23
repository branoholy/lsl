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

#ifndef LSL_PROBABILITY_NORMALDISTRIBUTION_HPP
#define LSL_PROBABILITY_NORMALDISTRIBUTION_HPP

namespace lsl {
namespace probability {

class NormalDistribution
{
private:
	double mean;
	double stdDev;

public:
	NormalDistribution(double mean = 0, double stdDev = 1);

	inline double getMean() const { return mean; }
	inline void setMean(double mean) { this->mean = mean; }
	inline double getStdDev() const { return stdDev; }
	inline void setStdDev(double stdDev) { this->stdDev = stdDev; }
	inline double getVariance() const { return this->stdDev * this->stdDev; }

	double operator()(double x) const;
};

}}

#endif // LSL_PROBABILITY_NORMALDISTRIBUTION_HPP
