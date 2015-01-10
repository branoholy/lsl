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

#include "lsl/gui/events/sizeevents.hpp"

using namespace std;
using namespace lsl::system;

namespace lsl {
namespace gui {
namespace events {

SizeEvents::SizeEvents(const wxWindow *window) : SizeEvents(window->GetEventHandler())
{
}

SizeEvents::SizeEvents(wxEvtHandler *evtHandler)
{
	evtHandler->Bind(wxEVT_SIZE, &SizeEvents::evtSize, this);
}

void SizeEvents::evtSize(wxSizeEvent &e)
{
	if(onSizeChanged.isEmpty()) e.Skip();
	else onSizeChanged(e);
}

}}}
