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

#include "lsl/visual/display2d.hpp"

using namespace std;
using namespace lsl::geom;

namespace lsl {
namespace visual {

Display2d::Display2d(const Vector2d& position, double zoom, double minZoom, double maxZoom) :
	zoom(1), minZoom(minZoom), maxZoom(maxZoom)
{
	this->position.set(position);
	setZoom(zoom);
	setZoomStep(2);
}

void Display2d::setZoom(double zoom)
{
	if(zoom >= minZoom && zoom <= maxZoom)
	{
		this->zoom = zoom;
	}
}

void Display2d::setZoomStep(double zoomStep)
{
	zoomInStep = zoomStep;
	zoomOutStep = 1 / zoomStep;
}

void Display2d::setZoomStep(double zoomInStep, double zoomOutStep)
{
	this->zoomInStep = zoomInStep;
	this->zoomOutStep = zoomOutStep;
}

void Display2d::transformToDisplay(const Vector2d& point, int& dx, int& dy)
{
	transformToDisplay(point[0], point[1], dx, dy, zoom);
}

void Display2d::transformToDisplay(double x, double y, int& dx, int& dy)
{
	transformToDisplay(x, y, dx, dy, zoom);
}

void Display2d::transformToDisplay(const Vector2d& point, int& dx, int& dy, double zoom)
{
	transformToDisplay(point[0], point[1], dx, dy, zoom);
}

void Display2d::transformToDisplay(double x, double y, int& dx, int& dy, double zoom)
{
	dx = lrint((x - position[0]) * zoom);
	dy = lrint((position[1] - y) * zoom);
}

void Display2d::transformFromDisplay(const wxPoint& point, double& x, double& y)
{
	transformFromDisplay(point.x, point.y, x, y);
}

void Display2d::transformFromDisplay(int dx, int dy, double& x, double& y)
{
	x = dx / zoom + position[0];
	y = position[1] - dy / zoom;
}

}}
