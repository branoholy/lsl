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

#ifndef LSL_GUI_EVENTS_SIZEEVENTS_HPP
#define LSL_GUI_EVENTS_SIZEEVENTS_HPP

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "lsl/system/event.hpp"

namespace lsl {
namespace gui {
namespace events {

class SizeEvents
{
private:
	void evtSize(wxSizeEvent &e);

public:
	system::Event<void(wxSizeEvent&)> onSizeChanged;

	SizeEvents(const wxWindow *window);
	SizeEvents(wxEvtHandler *evtHandler);
};

}}}

#endif // LSL_GUI_EVENTS_SIZEEVENTS_HPP
