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

#include <wx/dcgraph.h>
#include <wx/rawbmp.h>
#include <wx/valnum.h>

#include "lsl/utils/arrayutils.hpp"
#include "lsl/utils/colorutils.hpp"
#include "lsl/utils/mathutils.hpp"
#include "lsl/utils/stringutils.hpp"

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
	initLeftSide(false), initSizeSet(false), axisSize(3000), transformationId(0),
	fNames{"tx", "ty", "phi"}, fValues{0, 0, 0}, fSteps{1, 1, 0.01},
	valuePrecision(12), stepPrecision(12), errorValuePrecision(12),
	frValues{nullptr, nullptr, nullptr}, frRows{0, 0, 0}, frColumns{0, 0, 0}
{
	setAxisAngle(MathUtils::PI / 2);

	onInit += bind(&LSLVisualizer2d::initGUI, this, _1);
}

LSLVisualizer2d::~LSLVisualizer2d()
{
	for(size_t i = 0; i < 3; i++)
	{
		ArrayUtils::delete2dArray(frValues[i], frRows[i]);
	}
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

void LSLVisualizer2d::addLidarLineCloud(const std::vector<LidarLine2>& lidarLineCloud)
{
	lidarLineClouds.push_back(lidarLineCloud);
}

void LSLVisualizer2d::showLeftSide()
{
	initLeftSide = true;
}

void LSLVisualizer2d::initGUI(Window *window)
{
	window->SetMinSize(wxSize(800, 450));
	window->Maximize();
	window->setExitOn(WXK_ESCAPE, 'Q');

	window->onCharHooked += [this, window](wxKeyEvent& keyEvent)
	{
		// TODO: Step by gradient

		if(keyEvent.GetKeyCode() == 'A' || keyEvent.GetKeyCode() == 'D' ||
		   keyEvent.GetKeyCode() == WXK_LEFT || keyEvent.GetKeyCode() == WXK_RIGHT ||
		   keyEvent.GetKeyCode() == WXK_UP || keyEvent.GetKeyCode() == WXK_DOWN)
		{
			window->TransferDataFromWindow();

			int index = 0; // 1, 2, 3
			switch (keyEvent.GetKeyCode())
			{
				case WXK_RIGHT: index = 1; break;
				case WXK_LEFT: index = -1; break;
				case WXK_UP: index = 2; break;
				case WXK_DOWN: index = -2; break;
				case 'A': index = 3; break;
				case 'D': index = -3; break;
			}

			int absIndex = abs(index);
			if(absIndex > 0) fValues[absIndex - 1] += (index / absIndex) * fSteps[absIndex - 1];

			window->TransferDataToWindow();
			refreshLLTError();
		}
		else keyEvent.Skip();
	};

	window->onLeftMouseClick += [this](wxMouseEvent&)
	{
		// Workaround: Remove focus of text controls when clicking into main sizer and do not make TAB traversal.
		repaintingPanel->SetFocus();

		for(size_t i = 0; i < 3; i++)
		{
			fValueCtrl[i]->SetSelection(0, 0);
			fStepCtrl[i]->SetSelection(0, 0);
		}
	};

	// Main sizer (splits window into left transfomation controls and right panel with points)
	mainSizer = new wxBoxSizer(wxHORIZONTAL);

	leftSizer = new wxBoxSizer(wxVERTICAL);
	if(initLeftSide) initLeftSideControls(window, leftSizer);
	mainSizer->Add(leftSizer, 0, wxLEFT | wxTOP, 10);

	// Panel with points
	repaintingPanel = new Panel(window);
	repaintingPanel->SetBackgroundColour(*wxWHITE);
	repaintingPanel->SetFocus();

	repaintingPanel->onRepaint += bind(&LSLVisualizer2d::repaint, this, _1);

	repaintingPanel->onSizeChanged += [this, window](wxSizeEvent& e)
	{
		if(window->IsMaximized() && !initSizeSet && repaintingPanel->IsShown())
		{
			wxSize newSize = e.GetSize();
			display.getPosition()[0] = -newSize.GetWidth() / 2;
			display.getPosition()[1] = newSize.GetHeight() / 2;

			initSizeSet = true;
		}

		e.Skip();
	};

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

	mainSizer->Add(repaintingPanel, 1, wxEXPAND | wxALL, 10);
	window->SetSizer(mainSizer);

	refreshLLTError();
}

void LSLVisualizer2d::initLeftSideControls(wxWindow *parent, wxSizer *parentSizer)
{
	int stepCtrlSize = 50;

	// Sizer for functions
	wxGridSizer *frSizer = new wxGridSizer(2, 2, 5, 5);
	frSizer->SetMinSize(wxSize(350, 350));

	parentSizer->Add(frSizer, 1, wxSHAPED);

	// Panels for functions
	for(size_t i = 0; i < 3; i++)
	{
		frPanels[i] = new Panel(parent);
		frPanels[i]->SetBackgroundColour(*wxBLACK);
		frPanels[i]->onRepaint += bind(&LSLVisualizer2d::frRepaint, this, i, _1, _2);
		frPanels[i]->onMouseClick += [this, parent, i](wxMouseEvent& e)
		{
			getParams(i, e.GetX(), e.GetY(), fValues[0], fValues[1], fValues[2]);

			parent->TransferDataToWindow();
			refreshLLTError();
		};

		frSizer->Add(frPanels[i], 1, wxEXPAND);
	}

	// Sizer for params controls
	wxGridBagSizer *paramsSizer = new wxGridBagSizer(4, 4);
	for(size_t i = 0; i < 3; i++)
	{
		paramsSizer->Add(new wxStaticText(parent, wxID_ANY, fNames[i] + ":"), wxGBPosition(i, 0), wxDefaultSpan, wxALIGN_CENTER_VERTICAL);

		fValueCtrl[i] = new wxTextCtrl(parent, wxID_ANY, StringUtils::toString(fValues[i], valuePrecision, false),
									   wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER,
									   wxFloatingPointValidator<double>(valuePrecision, &fValues[i], wxNUM_VAL_NO_TRAILING_ZEROES));
		fValueCtrl[i]->SetMinSize(wxSize(1, wxDefaultCoord));
		fValueCtrl[i]->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&) { refreshLLTError(); });
		paramsSizer->Add(fValueCtrl[i], wxGBPosition(i, 1), wxDefaultSpan, wxEXPAND);

		fStepCtrl[i] = new wxTextCtrl(parent, wxID_ANY, StringUtils::toString(fSteps[i], stepPrecision, false),
									  wxDefaultPosition, wxSize(stepCtrlSize, wxDefaultCoord), wxTE_PROCESS_ENTER,
									  wxFloatingPointValidator<double>(stepPrecision, &fSteps[i], wxNUM_VAL_NO_TRAILING_ZEROES));
		fStepCtrl[i]->Bind(wxEVT_TEXT_ENTER, [this, i](wxCommandEvent&) { refreshLLTError(); });
		paramsSizer->Add(fStepCtrl[i], wxGBPosition(i, 2));
	}
	refreshToolTips();

	paramsSizer->Add(new wxStaticText(parent, wxID_ANY, "error:"), wxGBPosition(3, 0));
	errorValueCtrl = new wxStaticText(parent, wxID_ANY, StringUtils::toString(0.0, errorValuePrecision), wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
	errorValueCtrl->SetMinSize(wxSize(1, wxDefaultCoord));
	errorValueCtrl->SetToolTip(StringUtils::toString(0.0, errorValuePrecision));
	paramsSizer->Add(errorValueCtrl, wxGBPosition(3, 1), wxGBSpan(1, 2), wxEXPAND);

	paramsSizer->Add(new wxStaticText(parent, wxID_ANY, "CF:"), wxGBPosition(4, 0));
	cfValueCtrl = new wxStaticText(parent, wxID_ANY, StringUtils::toString(0.0, errorValuePrecision), wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
	cfValueCtrl->SetMinSize(wxSize(1, wxDefaultCoord));
	cfValueCtrl->SetToolTip(StringUtils::toString(0.0, errorValuePrecision));
	paramsSizer->Add(cfValueCtrl, wxGBPosition(4, 1), wxGBSpan(1, 2), wxEXPAND);

	paramsSizer->Add(new wxStaticText(parent, wxID_ANY, "CAF:"), wxGBPosition(5, 0));
	cafValueCtrl = new wxStaticText(parent, wxID_ANY, StringUtils::toString(0.0, errorValuePrecision), wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
	cafValueCtrl->SetMinSize(wxSize(1, wxDefaultCoord));
	cafValueCtrl->SetToolTip(StringUtils::toString(0.0, errorValuePrecision));
	paramsSizer->Add(cafValueCtrl, wxGBPosition(5, 1), wxGBSpan(1, 2), wxEXPAND);

	paramsSizer->AddGrowableCol(1, 1);

	frSizer->Insert(2, paramsSizer, 0, wxEXPAND);

	// Max error value
	wxFlexGridSizer *leftDownSizer = new wxFlexGridSizer(2, 5, 5);
	leftDownSizer->Add(new wxStaticText(parent, wxID_ANY, "Max error:"), 0, wxALIGN_CENTER_VERTICAL);

	wxBoxSizer *maxErrorSizer = new wxBoxSizer(wxHORIZONTAL);
	maxErrorValueCtrl = new wxTextCtrl(parent, wxID_ANY, "5000", wxDefaultPosition, wxSize(50, wxDefaultCoord), wxTE_PROCESS_ENTER);
	maxErrorValueCtrl->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent&)
	{
		long value;
		if(maxErrorValueCtrl->GetValue().ToLong(&value))
		{
			maxErrorSlider->SetValue(value);
			for(size_t i = 0; i < 3; i++) frPanels[i]->Refresh();
		}
	});
	maxErrorSizer->Add(maxErrorValueCtrl, 0, wxALIGN_CENTER_VERTICAL);

	maxErrorSlider = new wxSlider(parent, wxID_ANY, 5000, 10, 50000);
	maxErrorSlider->Bind(wxEVT_SLIDER, [this](wxCommandEvent&)
	{
		maxErrorValueCtrl->SetValue(to_string(maxErrorSlider->GetValue()));
		for(size_t i = 0; i < 3; i++) frPanels[i]->Refresh();
	});

	maxErrorSizer->Add(maxErrorSlider, 1, wxEXPAND);
	leftDownSizer->Add(maxErrorSizer, 1, wxEXPAND);

	// TODO: Add show points/lines/areas ticks

	leftDownSizer->AddGrowableCol(1, 1);
	parentSizer->Add(leftDownSizer, 1, wxEXPAND);
}

void LSLVisualizer2d::repaint(wxPaintDC& pdc)
{
	wxGCDC dc(pdc);

	// Paint
	dc.SetBrush(*wxLIGHT_GREY_BRUSH);

	// Draw error areas
	if(errorAreas.size() % 4 == 0)
	{
		wxPoint polyPoints[4];
		size_t errorAreasSize = errorAreas.size() / 4;
		for(size_t i = 0; i < errorAreasSize; i++)
		{
			for(size_t j = 0; j < 4; j++)
			{
				size_t index = i * 4 + j;
				int dx, dy;
				display.transformToDisplay(errorAreas.at(index), dx, dy);
				polyPoints[j] = wxPoint(dx, dy);
			}

			dc.DrawPolygon(4, polyPoints);
		}
	}

	dc.SetBrush(*wxWHITE_BRUSH);

	// Draw axis, rules
	drawAxis(dc);
	// drawRulers(dc);

	size_t coloursSize = sizeof(colours) / sizeof(wxColour*);

	// Draw point clouds
	/*
	size_t cloudsSize = pointClouds.size();
	for(size_t i = 0; i < cloudsSize; i++)
	{
		const auto *cloud = pointClouds[i];
		const wxColour *colour = colours[i % coloursSize];

		if(i == 1) drawPointCloud(dc, cloud, phiValue, txValue, tyValue, *colour);
		else drawPointCloud(dc, cloud, *colour);
	}
	*/

	// Draw lines
	size_t linesSize = lines.size();
	for(size_t i = 0; i < linesSize; i++)
	{
		const Line2& line = lines[i];
		drawLine(dc, line, *colours[i % coloursSize]);
		// drawLine(dc, line, 1, *wxRED);
	}

	// Draw lidar lines
	drawLidarLineCloud(dc, lidarLines, 1);

	// Draw lidar lines clouds
	size_t lidarLineCloudsSize = lidarLineClouds.size();
	for(size_t i = 0; i < lidarLineCloudsSize; i++)
	{
		const vector<LidarLine2>& lineCloud = lidarLineClouds[i];
		const wxColour *colour = colours[i % coloursSize];

		if(i == 1)
		{
			// drawLidarLineCloud(dc, lineCloud, phiValue, txValue, tyValue, *colour);

			vector<LidarLine2> transformedLineCloud = lineCloud;
			LidarLine2::transform(transformedLineCloud, fValues[2], fValues[0], fValues[1]);
			drawLidarLineCloud(dc, transformedLineCloud, *colour, 2);
		}
		else drawLidarLineCloud(dc, lineCloud, *colour, 2);
	}
}

void LSLVisualizer2d::drawAxis(wxDC& dc)
{
	const wxPen *normalPen = wxLIGHT_GREY_PEN;
	const wxPen *widePen = wxThePenList->FindOrCreatePen(*wxLIGHT_GREY, 2);

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

void LSLVisualizer2d::drawPoint(wxDC& dc, const Vector2d& point, const wxColour& colour, size_t size)
{
	drawPoint(dc, point, colour, colour, size);
}

void LSLVisualizer2d::drawPoint(wxDC& dc, const Vector2d& point, const wxColour& brushColour, const wxColour& penColour, size_t size)
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

void LSLVisualizer2d::drawLine(wxDC& dc, const Line2& line, const wxColour& colour, size_t size)
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

void LSLVisualizer2d::drawLidarLine(wxDC& dc, const LidarLine2& lidarLine, const wxColour& colour, size_t size, size_t endPointsSize)
{
	int dx1, dy1, dx2, dy2;
	display.transformToDisplay(lidarLine.getEndPointA(), dx1, dy1);
	display.transformToDisplay(lidarLine.getEndPointB(), dx2, dy2);

	wxPenStyle style = wxPENSTYLE_SOLID;
	if(!lidarLine.isVisible()) style = wxPENSTYLE_LONG_DASH;

	dc.SetPen(*wxThePenList->FindOrCreatePen(colour, size, style));
	dc.DrawLine(dx1, dy1, dx2, dy2);

	drawPoint(dc, lidarLine.getEndPointA(), colour, *wxBLACK, endPointsSize);

	dc.SetPen(*wxBLACK_PEN);
	dc.SetBrush(*wxTheBrushList->FindOrCreateBrush(colour));
	dc.DrawCircle(dx2, dy2, endPointsSize);
}

void LSLVisualizer2d::drawPointCloud(wxDC& dc, const PointCloud<Vector2d> *pointCloud, const wxColour& colour, size_t size)
{
	for(const Vector2d& point : *pointCloud)
	{
		drawPoint(dc, point, size, colour);
	}
}

void LSLVisualizer2d::drawPointCloud(wxDC& dc, const PointCloud<Vector2d> *pointCloud, double phi, double tx, double ty, const wxColour& colour, size_t size)
{
	for(Vector2d point : *pointCloud)
	{
		point.transform2D(phi, tx, ty);
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
		drawLidarLine(dc, lidarLine, *colours[i % coloursSize], size);
	}
}

void LSLVisualizer2d::drawLidarLineCloud(wxDC& dc, const vector<LidarLine2>& lineCloud, double phi, double tx, double ty, size_t size)
{
	size_t coloursSize = sizeof(colours) / sizeof(wxColour*);
	size_t cloudSize = lineCloud.size();

	for(size_t i = 0; i < cloudSize; i++)
	{
		LidarLine2 lidarLine = lineCloud[i];
		lidarLine.transform(phi, tx, ty);
		drawLidarLine(dc, lidarLine, *colours[i % coloursSize], size);
	}
}

void LSLVisualizer2d::drawLidarLineCloud(wxDC& dc, const vector<LidarLine2>& lineCloud, const wxColour& colour, size_t size)
{
	for(const LidarLine2& lidarLine : lineCloud)
	{
		drawLidarLine(dc, lidarLine, colour, size);
	}
}

void LSLVisualizer2d::drawLidarLineCloud(wxDC& dc, const vector<LidarLine2>& lineCloud, double phi, double tx, double ty, const wxColour& colour, size_t size)
{
	for(LidarLine2 lidarLine : lineCloud)
	{
		lidarLine.transform(phi, tx, ty);
		drawLidarLine(dc, lidarLine, colour, size);
	}
}

void LSLVisualizer2d::frRepaint(size_t i, wxPaintDC &pdc, wxPaintEvent&)
{
	wxGCDC dc(pdc);

	wxBitmap bitmap(frColumns[i], frRows[i], 24);
	wxNativePixelData bitmapData(bitmap);

	wxNativePixelData::Iterator it(bitmapData);
	for(size_t y = 0; y < frRows[i]; y++)
	{
		double *row = frValues[i][y];
		wxNativePixelData::Iterator rowIt = it;

		for(size_t x = 0; x < frColumns[i]; x++, it++)
		{
			double value01 = MathUtils::to01(row[x], 0, maxErrorSlider->GetValue());

			int r, g, b;
			ColorUtils::range2rgb(value01, r, g, b);

			it.Red() = r;
			it.Green() = g;
			it.Blue() = b;
		}

		it = rowIt;
		it.OffsetY(bitmapData, 1);
	}

	dc.DrawBitmap(bitmap, 0, 0);

	dc.SetPen(*wxBLACK_PEN);
	dc.SetBrush(*wxTRANSPARENT_BRUSH);
	dc.DrawCircle(frColumns[i] / 2, frRows[i] / 2, 5);
}

void LSLVisualizer2d::refreshLLTError()
{
	if(lidarLineClouds.size() == 2)
	{
		getWindow()->TransferDataFromWindow();
		errorAreas = llt.errorAreas(lidarLineClouds.at(0), lidarLineClouds.at(1), fValues[2], fValues[0], fValues[1]);

		double values[3];
		values[0] = llt.errorTransform(lidarLineClouds.at(0), lidarLineClouds.at(1), fValues[2], fValues[0], fValues[1]);
		values[1] = llt.getCoverFactor();
		values[2] = llt.getCoverAngleFactor();

		wxStaticText *ctrls[] = {errorValueCtrl, cfValueCtrl, cafValueCtrl};
		for(size_t i = 0; i < 3; i++)
		{
			wstring strValue;
			if(values[i] == numeric_limits<double>::max()) strValue = L"DOUBLE_MAX";
			else
			{
				strValue = WStringUtils::toString(values[i], errorValuePrecision);

				if(i == 2)
				{
					strValue = WStringUtils::toString(MathUtils::toDegrees(values[i]), 3, true) + L"° (" + strValue + ')';
				}
			}

			ctrls[i]->SetLabel(strValue);
			ctrls[i]->SetToolTip(strValue);
		}

		repaintingPanel->Refresh();
		refreshLLTFunc();
		refreshToolTips();
	}
}

void LSLVisualizer2d::refreshLLTFunc()
{
	for(size_t i = 0; i < 3; i++) refreshLLTFunc(i);
}

void LSLVisualizer2d::refreshLLTFunc(size_t i)
{
	ArrayUtils::delete2dArray(frValues[i], frRows[i]);

	int width, height;
	frPanels[i]->GetSize(&width, &height);
	size_t ws = frRows[i] = height;
	size_t hs = frColumns[i] = width;

	double **values = frValues[i] = ArrayUtils::create2dArray<double>(height, width);
	for(size_t y = 0; y < hs; y++)
	{
		double *row = values[y];
		for(size_t x = 0; x < ws; x++)
		{
			double tx, ty, phi;
			getParams(i, x, y, tx, ty, phi);
			row[x] = llt.errorTransform(lidarLineClouds.at(0), lidarLineClouds.at(1), phi, tx, ty);
		}
	}

	frPanels[i]->Refresh();
}

void LSLVisualizer2d::refreshToolTips()
{
	for(size_t i = 0; i < 3; i++)
	{
		fValueCtrl[i]->SetToolTip(fNames[i] + " = " + StringUtils::toString(fValues[i], valuePrecision));
		fStepCtrl[i]->SetToolTip(fNames[i] + " step = " + StringUtils::toString(fSteps[i], stepPrecision));
	}
}

void LSLVisualizer2d::getParams(size_t i, size_t x, size_t y, double &tx, double &ty, double &phi) const
{
	int width, height;
	frPanels[i]->GetSize(&width, &height);
	size_t w2 = width / 2;
	size_t h2 = height / 2;

	if(i == 0)
	{
		tx = fValues[0];
		ty = fValues[1] + int(h2 - y) * fSteps[1];
		phi = fValues[2] + int(w2 - x) * fSteps[2];
	}
	else if(i == 1)
	{
		tx = fValues[0] + int(x - w2) * fSteps[0];
		ty = fValues[1] + int(h2 - y) * fSteps[1];
		phi = fValues[2];
	}
	else if(i == 2)
	{
		tx = fValues[0] + int(x - w2) * fSteps[0];
		ty = fValues[1];
		phi = fValues[2] + int(y - h2) * fSteps[2];
	}
}

}}
