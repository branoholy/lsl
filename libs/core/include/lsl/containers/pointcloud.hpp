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

#ifndef LSL_CONTAINERS_POINTCLOUD_HPP
#define LSL_CONTAINERS_POINTCLOUD_HPP

#include <iostream>
#include <vector>

#include "lsl/geom/transformable.hpp"

namespace lsl {
namespace containers {

template<typename ScalarT, int dim>
class PointCloud : public std::vector<geom::Matrix<ScalarT, dim + 1, 1>>, public geom::Transformable<ScalarT, dim>
{
public:
	typedef geom::Matrix<ScalarT, dim + 1, 1> Point; // x, y, w;;; x, y, z, w
	typedef typename geom::Transformable<ScalarT, dim>::Location Location;
	typedef typename geom::Transformable<ScalarT, dim>::Transformation Transformation;
	typedef geom::Vector3i Color;

private:
	std::size_t id;
	Location realLocation;
	Location odomLocation;
	Location correctedLocation;

	Color realColor, unrealColor;
	std::size_t realPointSize, unrealPointSize;

public:
	typedef ScalarT ScalarType;
	static const int dimension;

	PointCloud();
	PointCloud(std::initializer_list<Point> data);
	PointCloud(const PointCloud& pointCloud);

	inline std::size_t getId() const { return id; }
	inline void setId(std::size_t id) { this->id = id; }

	inline Location& getRealLocation() { return realLocation; }
	inline const Location& getRealLocation() const { return realLocation; }

	inline Location& getOdomLocation() { return odomLocation; }
	inline const Location& getOdomLocation() const { return odomLocation; }

	inline Location& getCorrectedLocation() { return correctedLocation; }
	inline const Location& getCorrectedLocation() const { return correctedLocation; }

	inline const Color& getRealColor() const { return realColor; }
	inline const Color& getUnrealColor() const { return unrealColor; }

	inline void setColors(const Color& realColor) { setColors(realColor, realColor); }
	inline void setColors(const Color& realColor, const Color& unrealColor)
	{
		this->realColor = realColor;
		this->unrealColor = unrealColor;
	}

	inline std::size_t getRealPointSize() const { return realPointSize; }
	inline std::size_t getUnrealPointSize() const { return unrealPointSize; }

	inline void setPointSizes(std::size_t realPointSize) { setPointSizes(realPointSize, realPointSize); }
	inline void setPointSizes(std::size_t realPointSize, std::size_t unrealPointSize)
	{
		this->realPointSize = realPointSize;
		this->unrealPointSize = unrealPointSize;
	}

	void zoom(double zoomSize);

	void correctIds();

	void getBounds(Point& low, Point& high, bool onlyReal = true) const;

	Point getCentroid() const;

	void transform(const Transformation& transformation);

	static void transformAll(std::vector<PointCloud>& clouds, const Transformation& transformation);
	static void transformAllToLocation(std::vector<PointCloud>& clouds, const Location& location);

	template<typename ScalarT_, int dim_>
	friend std::ostream& operator<<(std::ostream& out, const PointCloud<ScalarT_, dim_>& pointCloud);
};

typedef PointCloud<double, 2> PointCloud2d;
typedef PointCloud<double, 3> PointCloud3d;

template<typename ScalarT, int dim>
const int PointCloud<ScalarT, dim>::dimension = dim;

template<typename ScalarT, int dim>
PointCloud<ScalarT, dim>::PointCloud() :
	id(-1), realLocation(Location::Zero()), odomLocation(Location::Zero()), correctedLocation(Location::Zero()),
	realColor(Color::Zero()), realPointSize(1)
{
}

template<typename ScalarT, int dim>
PointCloud<ScalarT, dim>::PointCloud(std::initializer_list<Point> data) : std::vector<Point>(data),
	id(-1), realLocation(Location::Zero()), odomLocation(Location::Zero()), correctedLocation(Location::Zero()),
	realColor(Color::Zero()), realPointSize(1)
{
}

template<typename ScalarT, int dim>
PointCloud<ScalarT, dim>::PointCloud(const PointCloud<ScalarT, dim>& pointCloud) : std::vector<Point>(pointCloud)
{
	id = pointCloud.id;
	realLocation = pointCloud.realLocation;
	odomLocation = pointCloud.odomLocation;
	correctedLocation = pointCloud.correctedLocation;
	realColor = pointCloud.realColor;
	realPointSize = pointCloud.realPointSize;
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::zoom(double zoomSize)
{
	Transformation zoomTransform = Transformation::Identity();
	for(std::size_t i = 0; i < dim; i++)
	{
		zoomTransform(i, i) = zoomSize;

		realLocation[i] *= zoomSize;
		odomLocation[i] *= zoomSize;
		correctedLocation[i] *= zoomSize;
	}

	transform(zoomTransform);
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::correctIds()
{
	std::sort(this->begin(), this->end(), [] (const Point& a, const Point& b)
	{
		return a.getAngle2D() < b.getAngle2D();
	});

	std::size_t size = this->size();
	for(std::size_t i = 0; i < size; i++)
	{
		this->at(i).setId(i);
	}
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::getBounds(Point& low, Point& high, bool onlyReal) const
{
	for(int d = 0; d < dim; d++)
	{
		low[d] = std::numeric_limits<double>::max();
		high[d] = std::numeric_limits<double>::min();
	}
	low[2] = 1;
	high[2] = 1;

	for(const Point& point : *this)
	{
		if(onlyReal && !point.realPoint) continue;

		for(int d = 0; d < dim; d++)
		{
			if(point[d] < low[d]) low[d] = point[d];
			if(point[d] > high[d]) high[d] = point[d];
		}
	}
}

template<typename ScalarT, int dim>
typename PointCloud<ScalarT, dim>::Point PointCloud<ScalarT, dim>::getCentroid() const
{
	Point centroid = Point::Zero();
	for(const Point& point : *this)
	{
		centroid += point;
	}
	centroid *= 1.0 / this->size();

	return centroid;
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::transform(const Transformation& transformation)
{
	for(Point& point : *this)
	{
		point = transformation * point;
	}
	correctIds();
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::transformAll(std::vector<PointCloud>& clouds, const Transformation& transformation)
{
	for(PointCloud& cloud : clouds)
	{
		cloud.transform(transformation);
	}
}

template<typename ScalarT, int dim>
void PointCloud<ScalarT, dim>::transformAllToLocation(std::vector<PointCloud>& clouds, const Location& location)
{
	transformAll(clouds, geom::Transformable<ScalarT, dim>::createTransformation(location));
}

template<typename ScalarT_, int dim_>
std::ostream& operator<<(std::ostream& out, const PointCloud<ScalarT_, dim_>& pointCloud)
{
	out << '[';

	if(!pointCloud.empty())
	{
		Eigen::IOFormat cleanFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "", ", ", "", "", "[", "]");

		out << pointCloud.front().format(cleanFormat);
		for(std::size_t i = 1; i < pointCloud.size(); i++)
		{
			out << ", " << pointCloud[i].format(cleanFormat);
		}
	}

	out << ']';

	return out;
}

}}

#endif // LSL_CONTAINERS_POINTCLOUD_HPP
