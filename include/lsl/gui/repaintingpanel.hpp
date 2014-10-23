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

#ifndef LSL_GUI_REPAINTINGPANEL_H
#define LSL_GUI_REPAINTINGPANEL_H

#include <functional>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

namespace lsl {
namespace gui {

class RepaintingPanel : public wxPanel
{
private:
	std::function<void(wxKeyEvent&)> onKeyUpMethod;
	std::function<void(wxMouseEvent&)> onMouseDownMethod;
	std::function<void(wxMouseEvent&)> onMouseDClickMethod;
	std::function<void(wxMouseEvent&)> onMouseMoveMethod;
	std::function<void(wxDC&, wxPaintEvent&)> onRepaintMethod;

public:
	RepaintingPanel(wxWindow *parent);

	void onKeyUp(wxKeyEvent& keyEvent);
	void onKeyDown(wxKeyEvent& keyEvent);
	void onMouseDown(wxMouseEvent& mouseEvent);
	void onMouseDClick(wxMouseEvent& mouseEvent);
	void onMouseMove(wxMouseEvent& mouseEvent);
	void onRepaint(wxPaintEvent& evt);

	inline void setOnKeyUpMethod(std::function<void(wxKeyEvent&)> onKeyUpMethod) { this->onKeyUpMethod = onKeyUpMethod; }
	inline void setOnMouseDownMethod(std::function<void(wxMouseEvent&)> onMouseDownMethod) { this->onMouseDownMethod = onMouseDownMethod; }
	inline void setOnMouseDClickMethod(std::function<void(wxMouseEvent&)> onMouseDClickMethod) { this->onMouseDClickMethod = onMouseDClickMethod; }
	inline void setOnMouseMoveMethod(std::function<void(wxMouseEvent&)> onMouseMoveMethod) { this->onMouseMoveMethod = onMouseMoveMethod; }
	inline void setOnRepaintMethod(std::function<void(wxDC&, wxPaintEvent&)> onRepaintMethod) { this->onRepaintMethod = onRepaintMethod; }

	wxDECLARE_EVENT_TABLE();
};

}}

#endif // LSL_GUI_REPAINTINGPANEL_H
