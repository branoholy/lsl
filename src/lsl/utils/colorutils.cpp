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

#include "lsl/utils/colorutils.hpp"

using namespace std;

namespace lsl {
namespace utils {

void ColorUtils::hsv2rgb(double h, double s, double v, double& r, double& g, double& b)
{
	double c = v * s;
	double h_ = h / (M_PI / 3);
	double x = c * (1 - abs(fmod(h_, 2) - 1));

	if(h_ >= 0 && h_ < 1)
	{
		r = c;
		g = x;
		b = 0;
	}
	else if(h_ >= 1 && h_ < 2)
	{
		r = x;
		g = c;
		b = 0;
	}
	else if(h_ >= 2 && h_ < 3)
	{
		r = 0;
		g = c;
		b = x;
	}
	else if(h_ >= 3 && h_ < 4)
	{
		r = 0;
		g = x;
		b = c;
	}
	else if(h_ >= 4 && h_ < 5)
	{
		r = x;
		g = 0;
		b = c;
	}
	else if(h_ >= 5 && h_ < 6)
	{
		r = c;
		g = 0;
		b = x;
	}
	else
	{
		r = g = b = 0;
	}

	double m = v - c;
	r += m;
	g += m;
	b += m;
}

void ColorUtils::hsv2rgb(double h, double s, double v, int& r, int& g, int& b)
{
	double rd, gd, bd;
	hsv2rgb(h, s, v, rd, gd, bd);
	r = rd * 255;
	g = gd * 255;
	b = bd * 255;
}

void ColorUtils::range2rgb(double value, double& r, double& g, double& b, double maxH)
{
	hsv2rgb(value * maxH, 1, 1, r, g, b);
}

void ColorUtils::range2rgb(double value, int& r, int& g, int& b, double maxH)
{
	hsv2rgb(value * maxH, 1, 1, r, g, b);
}

}}
