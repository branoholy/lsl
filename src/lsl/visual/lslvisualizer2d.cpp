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

#include "lsl/visual/lslvisualizer2d.hpp"

#include "lsl/utils/mathutils.hpp"

using namespace std;
using namespace std::placeholders;

using namespace lsl::containers;
using namespace lsl::geom;
using namespace lsl::gui;
using namespace lsl::system;
using namespace lsl::utils;

namespace lsl {
namespace visual {

const wxColour *LSLVisualizer2d::colours[] = {wxBLACK, wxRED, wxGREEN, wxBLUE};

LSLVisualizer2d::LSLVisualizer2d(const string& title, const wxSize& windowSize) : LSLApp(title, windowSize),
	initSizeSet(false), axisSize(3000)
{
	setAxisAngle(MathUtils::PI / 2);

	onInit += bind(&LSLVisualizer2d::initGUI, this, _1);
}

void LSLVisualizer2d::setAxisAngle(double axisAngle)
{
	this->axisAngle = axisAngle;
	axisAngleCos = cos(axisAngle);
	axisAngleSin = sin(axisAngle);
}

void LSLVisualizer2d::addPointCloud(const containers::PointCloud<Vector2d> *pointCloud)
{
	pointClouds.push_back(pointCloud);
}

void LSLVisualizer2d::addLine(const Line2& line)
{
	lines.push_back(line);
}

void LSLVisualizer2d::addLidarLine(const LidarLine2& lidarLine)
{
	lidarLines.push_back(lidarLine);
}

void LSLVisualizer2d::initGUI(Window *window)
{
	window->Maximize();
	window->setExitOn(WXK_ESCAPE, 'Q');

	window->onSizeChanged += [this, window](wxSizeEvent& e)
	{
		if(window->IsMaximized() && !initSizeSet)
		{
			wxSize newSize = window->GetClientSize();
			display.getPosition()[0] = -newSize.GetWidth() / 2;
			display.getPosition()[1] = newSize.GetHeight() / 2;
			initSizeSet = true;
		}

		e.Skip();
	};

	panel = new RepaintingPanel(window);
	panel->SetBackgroundColour(*wxWHITE);

	panel->onRepaint += bind(&LSLVisualizer2d::repaint, this, _1);

	panel->onLeftMouseDown += [this](wxMouseEvent& e)
	{
		mouseMoveStart = e.GetPosition();
	};

	panel->onMouseMove += [this](wxMouseEvent& e)
	{
		if(e.LeftIsDown())
		{
			display.getPosition()[0] -= (e.GetX() - mouseMoveStart.x) / display.getZoom();
			display.getPosition()[1] += (e.GetY() - mouseMoveStart.y) / display.getZoom();
			mouseMoveStart = e.GetPosition();

			panel->Refresh();
		}
	};

	panel->onMouseDoubleClick += [this](wxMouseEvent& mouseEvent)
	{
		if(mouseEvent.LeftDClick() || mouseEvent.RightDClick())
		{
			int width = 0;
			int height = 0;
			panel->GetClientSize(&width, &height);

			double clickX, clickY;
			display.transformFromDisplay(mouseEvent.GetPosition(), clickX, clickY);

			if(mouseEvent.LeftDClick()) display.zoomIn();
			else display.zoomOut();

			double zeroX = clickX - (width / display.getZoom()) / 2.0;
			double zeroY = clickY + (height / display.getZoom()) / 2.0;
			display.getPosition()[0] = zeroX;
			display.getPosition()[1] = zeroY;

			panel->Refresh();
		}
	};
}

void LSLVisualizer2d::repaint(wxDC& dc)
{
	dc.SetBrush(*wxWHITE_BRUSH);

	drawAxis(dc);

	size_t coloursSize = sizeof(colours) / sizeof(wxColour*);

	// Draw point clouds
	size_t cloudsSize = pointClouds.size();
	for(size_t i = 0; i < cloudsSize; i++)
	{
		const auto *cloud = pointClouds[i];
		drawPointCloud(dc, cloud, 1, *colours[i % coloursSize]);
	}

	// Draw lines
	size_t linesSize = lines.size();
	for(size_t i = 0; i < linesSize; i++)
	{
		const Line2& line = lines[i];
		drawLine(dc, line, 1, *colours[i % coloursSize]);
	}

	// Draw lidar lines
	size_t lidarLinesSize = lidarLines.size();
	for(size_t i = 0; i < lidarLinesSize; i++)
	{
		const LidarLine2& lidarLine = lidarLines[i];
		drawLidarLine(dc, lidarLine, 1, *colours[i % coloursSize]);
	}
}

void LSLVisualizer2d::drawAxis(wxDC& dc)
{
	const wxPen *normalPen = wxBLACK_PEN;
	const wxPen *widePen = wxThePenList->FindOrCreatePen(*wxBLACK, 2);

	Vector2d axisPoint1({-axisSize / display.getZoom(), 0.0});
	Vector2d axisPoint2({axisSize / display.getZoom(), 0.0});

	for(double angle = 0; angle < MathUtils::PI; angle += axisAngle)
	{
		if(angle == 0 || abs(angle - MathUtils::PI__TWO) < numeric_limits<double>::epsilon()) dc.SetPen(*widePen);
		else dc.SetPen(*normalPen);

		int x1, y1, x2, y2;
		display.transformToDisplay(axisPoint1, x1, y1);
		display.transformToDisplay(axisPoint2, x2, y2);

		dc.DrawLine(x1, y1, x2, y2);

		axisPoint1.rotate2D(axisAngleCos, axisAngleSin);
		axisPoint2.rotate2D(axisAngleCos, axisAngleSin);
	}

	dc.SetPen(*normalPen);
}

void LSLVisualizer2d::drawPoint(wxDC& dc, const Vector2d& point, size_t size, const wxColour& colour)
{
	double halfSize = 0.5 * size;

	int dx, dy;
	display.transformToDisplay(point, dx, dy);
	dx -= halfSize;
	dy -= halfSize;

	dc.SetPen(*wxThePenList->FindOrCreatePen(colour));
	dc.SetBrush(*wxTheBrushList->FindOrCreateBrush(colour));
	dc.DrawRectangle(dx, dy, size, size);
}

void LSLVisualizer2d::drawLine(wxDC& dc, const Line2& line, size_t size, const wxColour& colour)
{
	// FIXME: Draw zoomed line

	double x1, y1, x2, y2;
	x1 = y1 = x2 = y2 = 0;
	if(abs(line.getA()) > numeric_limits<double>::epsilon())
	{
		y1 = axisSize / display.getZoom();
		x1 = line.getX(y1);
		y2 = -y1;
		x2 = line.getX(y2);
	}
	else if(abs(line.getB()) > numeric_limits<double>::epsilon())
	{
		x1 = axisSize / display.getZoom();
		y1 = line.getY(x1);
		x2 = -x1;
		y2 = line.getY(x2);
	}

	int dx1, dy1, dx2, dy2;
	display.transformToDisplay(x1, y1, dx1, dy1);
	display.transformToDisplay(x2, y2, dx2, dy2);

	dc.SetPen(*wxThePenList->FindOrCreatePen(colour, size, wxPENSTYLE_DOT));
	dc.DrawLine(dx1, dy1, dx2, dy2);
}

void LSLVisualizer2d::drawLidarLine(wxDC& dc, const LidarLine2& lidarLine, size_t size, const wxColour& colour)
{
	int dx1, dy1, dx2, dy2;
	display.transformToDisplay(lidarLine.getEndPointA(), dx1, dy1);
	display.transformToDisplay(lidarLine.getEndPointB(), dx2, dy2);

	dc.SetPen(*wxThePenList->FindOrCreatePen(colour, size));
	dc.DrawLine(dx1, dy1, dx2, dy2);

	drawPoint(dc, lidarLine.getEndPointA(), 8 * size, colour);
	drawPoint(dc, lidarLine.getEndPointB(), 8 * size, colour);
}

void LSLVisualizer2d::drawPointCloud(wxDC& dc, const containers::PointCloud<Vector2d> *pointCloud, size_t size, const wxColour& colour)
{
	for(const Vector2d& point : *pointCloud)
	{
		drawPoint(dc, point, size, colour);
	}
}

}}
