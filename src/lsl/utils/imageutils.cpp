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

#include "lsl/utils/imageutils.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/colorutils.hpp"

using namespace std;

namespace lsl {
namespace utils {

void ImageUtils::saveToImage(const string& filename, double ** const data, size_t rowCount, size_t columnCount)
{
	double minValue, maxValue;
	ArrayUtils::getMinMaxValues(data, rowCount, columnCount, minValue, maxValue);
	saveToImage(filename, data, rowCount, columnCount, minValue, maxValue);
}

void ImageUtils::saveToImage(const string& filename, double ** const data, size_t rowCount, size_t columnCount, double maxValue)
{
	saveToImage(filename, data, rowCount, columnCount, 0, maxValue);
}

void ImageUtils::saveToImage(const string& filename, double ** const data, size_t rowCount, size_t columnCount, double minValue, double maxValue)
{
	double one__range = 1.0 / (maxValue - minValue);

	cv::Mat image(rowCount, columnCount, CV_8UC3);
	for(size_t r = 0; r < rowCount; r++)
	{
		for(size_t c = 0; c < columnCount; c++)
		{
			int R, G, B;
			size_t row = rowCount - r - 1;
			double value = data[row][c];
			if(value < minValue) value = minValue;
			if(value > maxValue) value = maxValue;

			if(value == 0)
			{
				R = G = B = 0;
			}
			else
			{
				double value01 = (value - minValue) * one__range;
				ColorUtils::range2rgb(value01, R, G, B);
			}

			cv::Vec3b& pixel = image.at<cv::Vec3b>(r, c);
			pixel[0] = B;
			pixel[1] = G;
			pixel[2] = R;
		}
	}

	cv::imwrite(filename, image);
}

}}
