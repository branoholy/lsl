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

#ifndef LSL_GUI_MOUSEEVENTS_HPP
#define LSL_GUI_MOUSEEVENTS_HPP

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "lsl/system/event.hpp"

namespace lsl {
namespace gui {

class MouseEvents
{
private:
	void evtLeftDown(wxMouseEvent& e);
	void evtMiddleDown(wxMouseEvent& e);
	void evtRightDown(wxMouseEvent& e);

	void evtLeftUp(wxMouseEvent& e);
	void evtMiddleUp(wxMouseEvent& e);
	void evtRightUp(wxMouseEvent& e);

	void evtLeftDClick(wxMouseEvent& e);
	void evtMiddleDClick(wxMouseEvent& e);
	void evtRightDClick(wxMouseEvent& e);

public:
	system::Event<void(wxMouseEvent&)> onMouseDown;
	system::Event<void(wxMouseEvent&)> onLeftMouseDown;
	system::Event<void(wxMouseEvent&)> onMiddleMouseDown;
	system::Event<void(wxMouseEvent&)> onRightMouseDown;

	system::Event<void(wxMouseEvent&)> onMouseUp;
	system::Event<void(wxMouseEvent&)> onLeftMouseUp;
	system::Event<void(wxMouseEvent&)> onMiddleMouseUp;
	system::Event<void(wxMouseEvent&)> onRightMouseUp;

	system::Event<void(wxMouseEvent&)> onMouseClick;
	system::Event<void(wxMouseEvent&)> onLeftMouseClick;
	system::Event<void(wxMouseEvent&)> onMiddleMouseClick;
	system::Event<void(wxMouseEvent&)> onRightMouseClick;

	system::Event<void(wxMouseEvent&)> onMouseDoubleClick;
	system::Event<void(wxMouseEvent&)> onLeftMouseDoubleClick;
	system::Event<void(wxMouseEvent&)> onMiddleMouseDoubleClick;
	system::Event<void(wxMouseEvent&)> onRightMouseDoubleClick;

	system::Event<void(wxMouseEvent&)> onMouseMove;

	MouseEvents(wxEvtHandler *evtHandler);
};

}}

#endif // LSL_GUI_MOUSEEVENTS_HPP
