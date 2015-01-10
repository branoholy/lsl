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

#include "lsl/gui/events/keyevents.hpp"

using namespace std;
using namespace lsl::system;

namespace lsl {
namespace gui {
namespace events {

KeyEvents::KeyEvents(wxEvtHandler *evtHandler)
{
	evtHandler->Bind(wxEVT_KEY_DOWN, &Event<void(wxKeyEvent&)>::operator(), &onKeyDown);
	evtHandler->Bind(wxEVT_KEY_UP, &KeyEvents::evtKeyUp, this);
	evtHandler->Bind(wxEVT_CHAR_HOOK, &KeyEvents::evtCharHooked, this);
}

void KeyEvents::evtKeyUp(wxKeyEvent& e)
{
	onKeyUp(e);
	e.Skip();
}

void KeyEvents::evtCharHooked(wxKeyEvent& e)
{
	wxWindow *focusedWindow = wxWindow::FindFocus();
	wxTextEntry *textEntry = dynamic_cast<wxTextEntry*>(focusedWindow);

	if(textEntry == nullptr)
	{
		if(onCharHooked.isEmpty()) e.Skip();
		else onCharHooked(e);
	}
	else e.Skip();
}

}}}
