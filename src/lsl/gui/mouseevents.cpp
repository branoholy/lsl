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

#include "lsl/gui/mouseevents.hpp"

using namespace std;
using namespace lsl::system;

namespace lsl {
namespace gui {

MouseEvents::MouseEvents(wxEvtHandler *evtHandler)
{
	evtHandler->Bind(wxEVT_LEFT_DOWN, &MouseEvents::evtLeftDown, this);
	evtHandler->Bind(wxEVT_MIDDLE_DOWN, &MouseEvents::evtMiddleDown, this);
	evtHandler->Bind(wxEVT_RIGHT_DOWN, &MouseEvents::evtRightDown, this);

	evtHandler->Bind(wxEVT_LEFT_DCLICK, &MouseEvents::evtLeftDClick, this);
	evtHandler->Bind(wxEVT_MIDDLE_DCLICK, &MouseEvents::evtMiddleDClick, this);
	evtHandler->Bind(wxEVT_RIGHT_DCLICK, &MouseEvents::evtRightDClick, this);

	evtHandler->Bind(wxEVT_MOTION, &Event<void(wxMouseEvent&)>::operator(), &onMouseMove);
}

void MouseEvents::evtLeftDown(wxMouseEvent &e)
{
	onMouseDown(e);
	onLeftMouseDown(e);
}

void MouseEvents::evtMiddleDown(wxMouseEvent &e)
{
	onMouseDown(e);
	onMiddleMouseDown(e);
}

void MouseEvents::evtRightDown(wxMouseEvent &e)
{
	onMouseDown(e);
	onRightMouseDown(e);
}

void MouseEvents::evtLeftDClick(wxMouseEvent &e)
{
	onMouseDoubleClick(e);
	onLeftMouseDoubleClick(e);
}

void MouseEvents::evtMiddleDClick(wxMouseEvent &e)
{
	onMouseDoubleClick(e);
	onMiddleMouseDoubleClick(e);
}

void MouseEvents::evtRightDClick(wxMouseEvent &e)
{
	onMouseDoubleClick(e);
	onRightMouseDoubleClick(e);
}

}}
