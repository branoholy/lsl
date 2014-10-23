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

#include "lsl/gui/repaintingpanel.hpp"

namespace lsl {
namespace gui {

RepaintingPanel::RepaintingPanel(wxWindow *parent) : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxWANTS_CHARS)
{
	onKeyUpMethod = nullptr;
	onMouseDownMethod = nullptr;
	onMouseMoveMethod = nullptr;
	onRepaintMethod = nullptr;
}

void RepaintingPanel::onKeyDown(wxKeyEvent& keyEvent)
{
	// Consume key down event to avoid focus change.
	switch(keyEvent.GetKeyCode())
	{
	case WXK_UP:
	case WXK_DOWN:
	case WXK_LEFT:
	case WXK_RIGHT:
		break;

	default:
		keyEvent.Skip();
	}
}

void RepaintingPanel::onKeyUp(wxKeyEvent& keyEvent)
{
	if(onKeyUpMethod != nullptr)
	{
		onKeyUpMethod(keyEvent);
	}
}

void RepaintingPanel::onMouseDown(wxMouseEvent& mouseEvent)
{
	if(onMouseDownMethod != nullptr)
	{
		onMouseDownMethod(mouseEvent);
	}
}

void RepaintingPanel::onMouseDClick(wxMouseEvent& mouseEvent)
{
	if(onMouseDClickMethod != nullptr)
	{
		onMouseDClickMethod(mouseEvent);
	}
}

void RepaintingPanel::onMouseMove(wxMouseEvent& mouseEvent)
{
	if(onMouseMoveMethod != nullptr)
	{
		onMouseMoveMethod(mouseEvent);
	}
}

void RepaintingPanel::onRepaint(wxPaintEvent& evt)
{
	if(onRepaintMethod != nullptr)
	{
		wxPaintDC dc(this);
		onRepaintMethod(dc, evt);
	}
}

wxBEGIN_EVENT_TABLE(RepaintingPanel, wxPanel)
EVT_KEY_DOWN(RepaintingPanel::onKeyDown)
EVT_KEY_UP(RepaintingPanel::onKeyUp)

EVT_LEFT_DOWN(RepaintingPanel::onMouseDown)
EVT_MIDDLE_DOWN(RepaintingPanel::onMouseDown)
EVT_RIGHT_DOWN(RepaintingPanel::onMouseDown)

EVT_LEFT_DCLICK(RepaintingPanel::onMouseDClick)
EVT_MIDDLE_DCLICK(RepaintingPanel::onMouseDClick)
EVT_RIGHT_DCLICK(RepaintingPanel::onMouseDClick)

EVT_MOTION(RepaintingPanel::onMouseMove)

EVT_PAINT(RepaintingPanel::onRepaint)
wxEND_EVENT_TABLE()

}}
