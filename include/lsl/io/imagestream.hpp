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

#ifndef LSL_IO_IMAGESTREAM_HPP
#define LSL_IO_IMAGESTREAM_HPP

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "lsl/io/stream.hpp"

namespace lsl {
namespace io {

template<typename PointCloudT>
class ImageStream : public Stream<PointCloudT>
{
protected:
	virtual void addPointCloud(const PointCloudT& pointCloud, cv::Mat& image, const typename PointCloudT::Point& lowBound) const;
	virtual void addAxes(cv::Mat& image, const typename PointCloudT::Point& lowBound, const typename PointCloudT::Point& highBound) const;
	virtual void saveImage(cv::Mat& image, const std::string& filePath) const;

	virtual PointCloudT loadData(std::istream& stream);
	virtual void saveData(const PointCloudT& pointCloud, std::ostream& stream);

public:
	double zoom;
	bool onlyReal;
	std::size_t padding;

	ImageStream();
	ImageStream(const std::string& filePath);

	virtual void save(const PointCloudT& pointCloud);
	virtual void save(const PointCloudT& pointCloud, const std::string& filePath);
	virtual void save(const PointCloudT& pointCloud, std::ostream& stream);

	virtual void saveAll(const std::vector<PointCloudT>& pointClouds);
	virtual void saveAll(const std::vector<PointCloudT>& pointClouds, const std::string& filePath);
};

template<typename PointCloudT>
ImageStream<PointCloudT>::ImageStream() : Stream<PointCloudT>(),
	zoom(1), onlyReal(true), padding(0)
{
}

template<typename PointCloudT>
ImageStream<PointCloudT>::ImageStream(const std::string& filePath) : Stream<PointCloudT>(filePath),
	zoom(1), onlyReal(true), padding(0)
{
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::save(const PointCloudT& pointCloud)
{
	return save(pointCloud, this->filePath);
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::save(const PointCloudT& pointCloud, const std::string& filePath)
{
	typename PointCloudT::Point lowBound, highBound;
	pointCloud.getBounds(lowBound, highBound, onlyReal);

	typename PointCloudT::Point size = (highBound - lowBound);
	for(std::size_t d = 0; d < PointCloudT::dimension; d++) size[d] += 2 * padding;

	cv::Mat image(size[1], size[0], CV_8UC3);
	image = cv::Scalar(255, 255, 255);

	addPointCloud(pointCloud, image, lowBound);
	addAxes(image, lowBound, highBound);

	saveImage(image, filePath);
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::save(const PointCloudT&, std::ostream&)
{
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::saveAll(const std::vector<PointCloudT>& pointClouds)
{
	saveAll(pointClouds, this->filePath);
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::saveAll(const std::vector<PointCloudT>& pointClouds, const std::string& filePath)
{
	PointCloudT boundCloud;
	boundCloud.reserve(2 * pointClouds.size());
	for(const auto& pointCloud : pointClouds)
	{
		typename PointCloudT::Point lowBound, highBound;
		pointCloud.getBounds(lowBound, highBound, onlyReal);

		boundCloud.push_back(lowBound);
		boundCloud.push_back(highBound);
	}

	typename PointCloudT::Point lowBound, highBound;
	boundCloud.getBounds(lowBound, highBound, false);

	typename PointCloudT::Point size = (highBound - lowBound);
	for(std::size_t d = 0; d < PointCloudT::dimension; d++) size[d] += 2 * padding;

	cv::Mat image(size[1], size[0], CV_8UC3);
	image = cv::Scalar(255, 255, 255);

	for(const auto& pointCloud : pointClouds)
	{
		addPointCloud(pointCloud, image, lowBound);
	}

	addAxes(image, lowBound, highBound);

	saveImage(image, filePath);
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::addPointCloud(const PointCloudT& pointCloud, cv::Mat& image, const typename PointCloudT::Point& lowBound) const
{
	for(const auto& point : pointCloud)
	{
		if(onlyReal && !point.realPoint) continue;

		typename PointCloudT::Color color;
		std::size_t pointSize;

		if(point.realPoint)
		{
			color = pointCloud.getRealColor();
			pointSize = pointCloud.getRealPointSize();
		}
		else
		{
			color = pointCloud.getUnrealColor();
			pointSize = pointCloud.getUnrealPointSize();
		}

		std::size_t x = point[0] - lowBound[0] + padding;
		std::size_t y = image.rows - (point[1] - lowBound[1] + padding);

		if(pointSize == 1)
		{
			cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
			for(std::size_t p = 0; p < 3; p++) pixel[p] = color[p];
		}
		else
		{
			cv::Rect rect(x - pointSize / 2, y - pointSize / 2, pointSize, pointSize);
			cv::rectangle(image, rect, cv::Scalar(color[0], color[1], color[2]), CV_FILLED);
		}
	}
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::addAxes(cv::Mat& image, const typename PointCloudT::Point& lowBound, const typename PointCloudT::Point& highBound) const
{
	if(lowBound[1] < 0 && highBound[1] > 0) cv::line(image, cv::Point(0, image.rows + lowBound[1] - padding), cv::Point(image.cols, image.rows + lowBound[1] - padding), cv::Scalar(0, 0, 0));
	if(lowBound[0] < 0 && highBound[0] > 0) cv::line(image, cv::Point(-lowBound[0] + padding, 0), cv::Point(-lowBound[0] + padding, image.rows), cv::Scalar(0, 0, 0));
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::saveImage(cv::Mat& image, const std::string& filePath) const
{
	if(zoom != 1)
	{
		cv::Mat zoomedImage;
		cv::resize(image, zoomedImage, cv::Size(), zoom, zoom, cv::INTER_NEAREST);
		image = zoomedImage;
	}

	cv::imwrite(filePath, image);
}

template<typename PointCloudT>
PointCloudT ImageStream<PointCloudT>::loadData(std::istream&)
{
	return PointCloudT();
}

template<typename PointCloudT>
void ImageStream<PointCloudT>::saveData(const PointCloudT&, std::ostream&)
{
}

}}

#endif // LSL_IO_IMAGESTREAM_HPP
