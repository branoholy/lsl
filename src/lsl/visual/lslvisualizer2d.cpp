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
#include "lsl/registration/llt.hpp"

using namespace std;
using namespace std::placeholders;

using namespace lsl::containers;
using namespace lsl::geom;
using namespace lsl::gui;
using namespace lsl::system;
using namespace lsl::utils;

namespace lsl {
namespace visual {

const wxColour *LSLVisualizer2d::colours[] = {wxBLACK, wxRED, wxGREEN, wxBLUE, wxCYAN};

LSLVisualizer2d::LSLVisualizer2d(const string& title, const wxSize& windowSize) : LSLApp(title, windowSize),
	transControls(false), initSizeSet(false), axisSize(3000), transformationId(0)
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

void LSLVisualizer2d::addPointCloud(containers::PointCloud<Vector2d> *pointCloud)
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

void LSLVisualizer2d::addLidarLines(const std::vector<LidarLine2>& lidarLines)
{
	this->lidarLines.insert(this->lidarLines.end(), lidarLines.begin(), lidarLines.end());
}

void LSLVisualizer2d::addLidarLineClouds(const std::vector<LidarLine2>& lidarLineCloud)
{
	lidarLineClouds.push_back(lidarLineCloud);
}

void LSLVisualizer2d::setTransformations(const std::vector<double>& transformations)
{
	this->transformations = transformations;
}

void LSLVisualizer2d::initGUI(Window *window)
{
	window->SetMinSize(wxSize(800, 450));
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

	// Main sizer (splits window into left transfomation controls and panel with points)
	mainSizer = new wxBoxSizer(wxHORIZONTAL);

	// Transformation controls
	transControlsSizer = new wxGridSizer(2, 2, 5, 5);
	transControlsSizer->SetMinSize(wxSize(250, 250));
	if(transControls) initTransContols(window);

	mainSizer->Add(transControlsSizer, 1, wxSHAPED | wxTOP | wxBOTTOM | wxLEFT, 10);

	// Panel with points
	repaintingPanel = new RepaintingPanel(window);
	repaintingPanel->SetBackgroundColour(*wxWHITE);
	repaintingPanel->SetFocus();

	repaintingPanel->onRepaint += bind(&LSLVisualizer2d::repaint, this, _1);

	repaintingPanel->onLeftMouseDown += [this](wxMouseEvent& e)
	{
		mouseMoveStart = e.GetPosition();
	};

	repaintingPanel->onMouseMove += [this](wxMouseEvent& e)
	{
		if(e.LeftIsDown())
		{
			display.getPosition()[0] -= (e.GetX() - mouseMoveStart.x) / display.getZoom();
			display.getPosition()[1] += (e.GetY() - mouseMoveStart.y) / display.getZoom();
			mouseMoveStart = e.GetPosition();

			repaintingPanel->Refresh();
		}
	};

	repaintingPanel->onMouseDoubleClick += [this](wxMouseEvent& mouseEvent)
	{
		if(mouseEvent.LeftDClick() || mouseEvent.RightDClick())
		{
			int width = 0;
			int height = 0;
			repaintingPanel->GetClientSize(&width, &height);

			double clickX, clickY;
			display.transformFromDisplay(mouseEvent.GetPosition(), clickX, clickY);

			if(mouseEvent.LeftDClick()) display.zoomIn();
			else display.zoomOut();

			double zeroX = clickX - (width / display.getZoom()) / 2.0;
			double zeroY = clickY + (height / display.getZoom()) / 2.0;
			display.getPosition()[0] = zeroX;
			display.getPosition()[1] = zeroY;

			repaintingPanel->Refresh();
		}
	};

	repaintingPanel->onKeyUp += [this](wxKeyEvent& keyEvent)
	{
		if(keyEvent.GetKeyCode() == 'P')
		{
			if(transformations.size() > 3 * transformationId)
			{
				double phi = transformations.at(3 * transformationId);
				double tx = transformations.at(3 * transformationId + 1);
				double ty = transformations.at(3 * transformationId + 2);
				transformationId++;

				if(pointClouds.size() > 1) pointClouds.at(1)->transform2D(phi, tx, ty);
				if(lidarLineClouds.size() > 1)
				{
					registration::LLT llt;
					LidarLine2::transform(lidarLineClouds.at(1), phi, tx, ty);
					llt.removeInvisible(lidarLineClouds.at(1));
				}

				repaintingPanel->Refresh();
			}
			else
			{
				cout << "No more transformations available." << endl;
			}
		}
	};

	mainSizer->Add(repaintingPanel, 3, wxEXPAND | wxALL, 10);

	window->addExitOnControl(repaintingPanel);
	window->onLeftMouseClick += [this](wxMouseEvent&)
	{
		// Workaround: Remove focus of text controls when clicking into main sizer and do not make TAB traversal.
		repaintingPanel->SetFocus();

		if(phiValue != nullptr) phiValue->SetSelection(0, 0);
		if(phiStep != nullptr) phiStep->SetSelection(0, 0);

		if(txValue != nullptr) txValue->SetSelection(0, 0);
		if(txStep != nullptr) txStep->SetSelection(0, 0);

		if(tyValue != nullptr) tyValue->SetSelection(0, 0);
		if(tyStep != nullptr) tyStep->SetSelection(0, 0);
	};

	window->SetSizer(mainSizer);
}

void LSLVisualizer2d::initTransContols(Window *window)
{
	// Panel for phi*ty function
	transPhiTy = new RepaintingPanel(window);
	transPhiTy->SetBackgroundColour(*wxBLUE);

	transControlsSizer->Add(transPhiTy, 1, wxEXPAND);

	// Panel for tx*ty
	transTxTy = new RepaintingPanel(window);
	transTxTy->SetBackgroundColour(*wxGREEN);

	transControlsSizer->Add(transTxTy, 1, wxEXPAND);

	// Sizer for params controls
	wxFlexGridSizer *paramsSizer = new wxFlexGridSizer(3, 3, 4, 4);
	paramsSizer->AddGrowableCol(1, 1);

	paramsSizer->Add(new wxStaticText(window, wxID_ANY, "phi:"), 0, wxALIGN_CENTER_VERTICAL);
	phiValue = new wxTextCtrl(window, wxID_ANY, "0.0");
	phiValue->SetMinSize(wxSize(1, wxDefaultCoord));
	paramsSizer->Add(phiValue, 1, wxEXPAND);

	phiStep = new wxTextCtrl(window, wxID_ANY, "0.01", wxDefaultPosition, wxSize(40, wxDefaultCoord));
	paramsSizer->Add(phiStep, 0);

	paramsSizer->Add(new wxStaticText(window, wxID_ANY, "tx:"), 0, wxALIGN_CENTER_VERTICAL);
	txValue = new wxTextCtrl(window, wxID_ANY, "0.0");
	txValue->SetMinSize(wxSize(1, wxDefaultCoord));
	paramsSizer->Add(txValue, 1, wxEXPAND);

	txStep = new wxTextCtrl(window, wxID_ANY, "1.0", wxDefaultPosition, wxSize(40, wxDefaultCoord));
	paramsSizer->Add(txStep, 0);

	paramsSizer->Add(new wxStaticText(window, wxID_ANY, "ty:"), 0, wxALIGN_CENTER_VERTICAL);
	tyValue = new wxTextCtrl(window, wxID_ANY, "0.0");
	tyValue->SetMinSize(wxSize(1, wxDefaultCoord));
	paramsSizer->Add(tyValue, 1, wxEXPAND);

	tyStep = new wxTextCtrl(window, wxID_ANY, "1.0", wxDefaultPosition, wxSize(40, wxDefaultCoord));
	paramsSizer->Add(tyStep, 0);

	transControlsSizer->Add(paramsSizer, 1, wxEXPAND);

	// Panel for tx*phi
	transTxPhi = new RepaintingPanel(window);
	transTxPhi->SetBackgroundColour(*wxRED);

	transControlsSizer->Add(transTxPhi, 1, wxEXPAND);

	window->addExitOnControl(transPhiTy);
	window->addExitOnControl(transTxTy);
	window->addExitOnControl(transTxPhi);
}

void LSLVisualizer2d::showTransControls()
{
	transControls = true;
}

void LSLVisualizer2d::repaint(wxDC& dc)
{
	dc.SetBrush(*wxWHITE_BRUSH);

	drawAxis(dc);
	// drawRulers(dc);

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
		// drawLine(dc, line, 1, *wxRED);
	}

	// Draw lidar lines
	drawLidarLineCloud(dc, lidarLines, 1);

	// Draw lidar lines clouds
	size_t lidarLineCloudsSize = lidarLineClouds.size();
	for(size_t i = 0; i < lidarLineCloudsSize; i++)
	{
		drawLidarLineCloud(dc, lidarLineClouds.at(i), 1, *colours[i % coloursSize]);
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

void LSLVisualizer2d::drawRulers(wxDC& dc)
{
	int w, h;
	repaintingPanel->GetSize(&w, &h);

	// x ruler
	dc.DrawLine(0, h - 5, w, h - 5);
	for(int x = 1; x < 10; x += 1)
	{
		dc.DrawLine(x / 10.0 * w, h, x / 10.0 * w, h - 5);
	}

	// y ruler
	dc.DrawLine(5, 0, 5, h);
	for(int y = 1; y < 10; y += 1)
	{
		dc.DrawLine(0, y / 10.0 * h, 5, y / 10.0 * h);
	}
}

void LSLVisualizer2d::drawPoint(wxDC& dc, const Vector2d& point, size_t size, const wxColour& colour)
{
	drawPoint(dc, point, size, colour, colour);
}

void LSLVisualizer2d::drawPoint(wxDC& dc, const Vector2d& point, size_t size, const wxColour& brushColour, const wxColour& penColour)
{
	double halfSize = 0.5 * size;

	int dx, dy;
	display.transformToDisplay(point, dx, dy);
	dx -= halfSize;
	dy -= halfSize;

	dc.SetBrush(*wxTheBrushList->FindOrCreateBrush(brushColour));
	dc.SetPen(*wxThePenList->FindOrCreatePen(penColour));

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

	for(const Vector2d& point : line.getPoints())
	{
		drawPoint(dc, point, size, colour);
	}
}

void LSLVisualizer2d::drawLidarLine(wxDC& dc, const LidarLine2& lidarLine, size_t size, const wxColour& colour)
{
	int dx1, dy1, dx2, dy2;
	display.transformToDisplay(lidarLine.getEndPointA(), dx1, dy1);
	display.transformToDisplay(lidarLine.getEndPointB(), dx2, dy2);

	wxPenStyle style = wxPENSTYLE_SOLID;
	if(!lidarLine.isVisible()) style = wxPENSTYLE_LONG_DASH;

	dc.SetPen(*wxThePenList->FindOrCreatePen(colour, size, style));
	dc.DrawLine(dx1, dy1, dx2, dy2);

	drawPoint(dc, lidarLine.getEndPointA(), 8 * size, colour, *wxBLACK);

	dc.SetPen(*wxThePenList->FindOrCreatePen(*wxBLACK, size, style));
	dc.SetBrush(*wxTheBrushList->FindOrCreateBrush(colour));
	dc.DrawCircle(dx2, dy2, 8 * size);
}

void LSLVisualizer2d::drawPointCloud(wxDC& dc, const PointCloud<Vector2d> *pointCloud, size_t size, const wxColour& colour)
{
	for(const Vector2d& point : *pointCloud)
	{
		drawPoint(dc, point, size, colour);
	}
}

void LSLVisualizer2d::drawLidarLineCloud(wxDC& dc, const vector<LidarLine2>& lineCloud, size_t size)
{
	size_t coloursSize = sizeof(colours) / sizeof(wxColour*);
	size_t cloudSize = lineCloud.size();

	for(size_t i = 0; i < cloudSize; i++)
	{
		const LidarLine2& lidarLine = lineCloud[i];
		drawLidarLine(dc, lidarLine, size, *colours[i % coloursSize]);
	}
}

void LSLVisualizer2d::drawLidarLineCloud(wxDC& dc, const vector<LidarLine2>& lineCloud, size_t size, const wxColour& colour)
{
	for(const LidarLine2& line : lineCloud)
	{
		drawLidarLine(dc, line, size, colour);
	}
}

}}
