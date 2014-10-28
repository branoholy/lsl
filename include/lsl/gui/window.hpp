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

#ifndef LSL_GUI_WINDOW_HPP
#define LSL_GUI_WINDOW_HPP

#include <functional>
#include <wx/frame.h>

#include "lsl/system/event.hpp"

namespace lsl {
namespace gui {

class Window : public wxFrame
{
private:
	bool exitOnEsc;
	void exitOnEscHook(wxKeyEvent& e);

public:
	system::Event<void(wxSizeEvent&)> onSizeChanged;
	system::Event<void(wxKeyEvent&)> onCharHooked;

	Window(const wxString& title, const wxSize& size);

	inline bool getExitOnEsc() const { return exitOnEsc; }
	inline void setExitOnEsc(bool exitOnEsc = true) { this->exitOnEsc = exitOnEsc; }
};

}}

#endif // LSL_GUI_WINDOW_HPP
