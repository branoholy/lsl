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

using namespace lsl::system;

namespace lsl {
namespace gui {

RepaintingPanel::RepaintingPanel(wxWindow *parent) : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxWANTS_CHARS), MouseEvents(GetEventHandler()), KeyEvents(GetEventHandler())
{
	Bind(wxEVT_PAINT, &RepaintingPanel::evtPaint, this);
}

/*
void RepaintingPanel::onKeyDown(wxKeyEvent& keyEvent)
{
	// TODO: Consume key down event to avoid focus change.
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
*/

void RepaintingPanel::evtPaint(wxPaintEvent& e)
{
	if(!onRepaint.isEmpty())
	{
		wxPaintDC dc(this);
		onRepaint(dc, e);
	}
}

}}
