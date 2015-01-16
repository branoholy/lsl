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

#ifndef LSL_VISUAL_LSLVISUALIZER2D_HPP
#define LSL_VISUAL_LSLVISUALIZER2D_HPP

#include <wx/gbsizer.h>

#include "lsl/containers/pointcloud.hpp"

#include "lsl/geom/vector.hpp"
#include "lsl/geom/lidarline2.hpp"
#include "lsl/geom/line2.hpp"

#include "lsl/gui/lslapp.hpp"
#include "lsl/gui/panel.hpp"

#include "lsl/registration/llt.hpp"

#include "display2d.hpp"

namespace lsl {
namespace visual {

class LSLVisualizer2d : public gui::LSLApp
{
private:
	static const wxColour *colours[];

	wxSizer *mainSizer;

	wxBoxSizer *leftSizer;
	bool initLeftSide;

	gui::Panel *frPanels[3];
	wxTextCtrl *fValueCtrl[3];
	wxTextCtrl *fStepCtrl[3];

	wxStaticText *errorValueCtrl;
	wxStaticText *cfValueCtrl;
	wxStaticText *cafValueCtrl;

	wxTextCtrl *maxErrorValueCtrl;
	wxSlider *maxErrorSlider;

	gui::Panel *repaintingPanel;

	Display2d display;
	bool initSizeSet;

	int axisSize;
	double axisAngle;
	double axisAngleCos;
	double axisAngleSin;

	wxPoint mouseMoveStart;

	std::vector<containers::PointCloud<geom::Vector2d>*> pointClouds;
	std::vector<geom::Line2> lines;
	std::vector<geom::LidarLine2> lidarLines;
	std::vector<std::vector<geom::LidarLine2>> lidarLineClouds;

	std::size_t transformationId;

	registration::LLT llt;

	std::string fNames[3];
	double fValues[3];
	double fSteps[3];

	int valuePrecision, stepPrecision;
	int errorValuePrecision;

	double **frValues[3];
	std::size_t frRows[3];
	std::size_t frColumns[3];

	std::vector<geom::Vector2d> errorAreas;

	void initGUI(gui::Window *window);
	void initLeftSideControls(wxWindow *parent, wxSizer *parentSizer);

	void repaint(wxPaintDC& pdc);

	void drawAxis(wxDC& dc);
	void drawRulers(wxDC& dc);

	void drawPoint(wxDC& dc, const geom::Vector2d& point, const wxColour& colour = *wxBLACK, std::size_t size = 1);
	void drawPoint(wxDC& dc, const geom::Vector2d& point, const wxColour& brushColour, const wxColour& penColour, std::size_t size = 1);

	void drawLine(wxDC& dc, const geom::Line2& line, const wxColour& colour = *wxBLACK, std::size_t size = 1);
	void drawLidarLine(wxDC& dc, const geom::LidarLine2& lidarLine, const wxColour& colour = *wxBLACK, std::size_t size = 1, std::size_t endPointsSize = 8);

	void drawPointCloud(wxDC& dc, const containers::PointCloud<geom::Vector2d> *pointCloud, const wxColour& colour = *wxBLACK, size_t size = 1);
	void drawPointCloud(wxDC& dc, const containers::PointCloud<geom::Vector2d> *pointCloud, double phi, double tx, double ty, const wxColour& colour = *wxBLACK, size_t size = 1);

	void drawLidarLineCloud(wxDC& dc, const std::vector<geom::LidarLine2>& lineCloud, std::size_t size = 1);
	void drawLidarLineCloud(wxDC& dc, const std::vector<geom::LidarLine2>& lineCloud, double phi, double tx, double ty, std::size_t size = 1);
	void drawLidarLineCloud(wxDC& dc, const std::vector<geom::LidarLine2>& lineCloud, const wxColour& colour, std::size_t size = 1);
	void drawLidarLineCloud(wxDC& dc, const std::vector<geom::LidarLine2>& lineCloud, double phi, double tx, double ty, const wxColour& colour, std::size_t size = 1);

	void frRepaint(std::size_t i, wxPaintDC& pdc, wxPaintEvent& e);

	void refreshLLTError();
	void refreshLLTFunc();
	void refreshLLTFunc(std::size_t i);
	void refreshToolTips();

	void getParams(std::size_t i, std::size_t x, std::size_t y, double& tx, double& ty, double& phi) const;

public:
	LSLVisualizer2d(const std::string& title = "LSL Visualizer 2D", const wxSize& windowSize = wxDefaultSize);
	~LSLVisualizer2d();

	inline double getAxisAngle() const { return axisAngle; }
	void setAxisAngle(double axisAngle);

	void addPointCloud(containers::PointCloud<geom::Vector2d>* pointCloud);
	void addLine(const geom::Line2& line);
	void addLidarLine(const geom::LidarLine2& lidarLine);
	void addLidarLines(const std::vector<geom::LidarLine2>& lidarLines);
	void addLidarLineCloud(const std::vector<geom::LidarLine2>& lidarLineCloud);

	void showLeftSide();
};

}}

#endif // LSL_VISUAL_LSLVISUALIZER2D_HPP
