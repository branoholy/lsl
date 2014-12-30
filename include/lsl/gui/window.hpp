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

#include <algorithm>
#include <vector>

#include <wx/frame.h>
#include <wx/window.h>

#include "mouseevents.hpp"

namespace lsl {
namespace gui {

class Window : public wxFrame, public MouseEvents
{
private:
	bool isExitOnKeysClear;
	std::vector<int> exitOnKeys;

	bool isExitOnControlsClear;
	std::vector<wxWindow*> exitOnControls;

	void exitOnEscHook(wxKeyEvent& e);

public:
	system::Event<void(wxSizeEvent&)> onSizeChanged;
	system::Event<void(wxKeyEvent&)> onCharHooked;

	Window(const wxString& title, const wxSize& size);

	inline bool getExitOn(int key) const { return std::find(exitOnKeys.begin(), exitOnKeys.end(), key) != exitOnKeys.end(); }
	void setExitOn(int key);

	template<typename ...Args>
	void setExitOn(int key, Args... keys);

	inline bool getExitOnControl(wxWindow *control) const { return std::find(exitOnControls.begin(), exitOnControls.end(), control) != exitOnControls.end(); }
	void setExitOnControl(wxWindow *control);

	template<typename ...Args>
	void setExitOnControl(wxWindow *control, Args... controls);

	void addExitOnControl(wxWindow *control);

	bool hasAnyExitOnControlFocus() const;
};

template<typename ...Args>
void Window::setExitOn(int key, Args... keys)
{
	if(!isExitOnKeysClear)
	{
		exitOnKeys.clear();
		isExitOnKeysClear = true;
	}

	exitOnKeys.push_back(key);
	setExitOn(keys...);
}

template<typename ...Args>
void Window::setExitOnControl(wxWindow *control, Args... controls)
{
	if(!isExitOnControlsClear)
	{
		exitOnControls.clear();
		isExitOnControlsClear = true;
	}

	exitOnControls.push_back(control);
	setExitOnControl(controls...);
}

}}

#endif // LSL_GUI_WINDOW_HPP
