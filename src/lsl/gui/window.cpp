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

#include "lsl/gui/window.hpp"

#include <iostream>

using namespace std;
using namespace lsl::system;

namespace lsl {
namespace gui {

Window::Window(const wxString& title, const wxSize& size) :
	wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, size),
	MouseEvents(GetEventHandler()), SizeEvents(this),
	isExitOnKeysClear(true)
{
	onKeyDown += bind(&Window::exitOn, this, placeholders::_1);
}

void Window::exitOn(wxKeyEvent& e)
{
	if(getExitOn(e.GetKeyCode())) Close(true);
	else e.Skip();
}

void Window::setExitOn(int key)
{
	if(!isExitOnKeysClear)
	{
		exitOnKeys.clear();
	}

	exitOnKeys.push_back(key);
	isExitOnKeysClear = false;
}

}}
