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

#ifndef LSL_VISUAL_DISPLAY2D_HPP
#define LSL_VISUAL_DISPLAY2D_HPP

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include <cmath>

#include "lsl/geom/vector.hpp"

namespace lsl {
namespace visual {

class Display2d
{
private:
	geom::Vector2d position;

	double zoom;
	double minZoom;
	double maxZoom;

	double zoomInStep;
	double zoomOutStep;

public:
	Display2d(const geom::Vector2d& position = {0, 0}, double zoom = 1, double minZoom = 1.0 / pow(2, 13), double maxZoom = 8192);

	inline geom::Vector2d& getPosition() { return position; }
	inline void setPosition(const geom::Vector2d& vector) { this->position.set(vector); }

	inline double getZoom() const { return zoom; }
	void setZoom(double zoom);

	inline void zoomIn() { setZoom(zoom * zoomInStep); }
	inline void zoomOut() { setZoom(zoom * zoomOutStep); }

	inline double getMinZoom() const { return minZoom; }
	inline void setMinZoom(double minZoom) { this->minZoom = minZoom; }

	inline double getMaxZoom() const { return maxZoom; }
	inline void setMaxZoom(double maxZoom) { this->maxZoom = maxZoom; }

	void setZoomStep(double zoomStep);
	void setZoomStep(double zoomInStep, double zoomOutStep);

	inline double getZoomInStep() const { return zoomInStep; }
	inline void setZoomInStep(double zoomInStep) { this->zoomInStep = zoomInStep; }

	inline double getZoomOutStep() const { return zoomOutStep; }
	inline void setZoomOutStep(double zoomOutStep) { this->zoomOutStep = zoomOutStep; }

	void transformToDisplay(const geom::Vector2d& point, int& dx, int& dy);
	void transformToDisplay(double x, double y, int& dx, int& dy);
	void transformToDisplay(const geom::Vector2d& point, int& dx, int& dy, double zoom);
	void transformToDisplay(double x, double y, int& dx, int& dy, double zoom);

	void transformFromDisplay(const wxPoint& point, double& x, double& y);
	void transformFromDisplay(int dx, int dy, double& x, double& y);
};

}}

#endif // LSL_VISUAL_DISPLAY2D_HPP
