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

using namespace std;

namespace lsl {
namespace gui {

Window::Window(const wxString& title, const wxSize& size) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, size)
{
	onSizeChangedMethod = nullptr;
}

void Window::keyCharHook(wxKeyEvent& event)
{
	if(event.GetKeyCode() == WXK_ESCAPE)
	{
		Close(true);
	}

	event.Skip();
}

void Window::setOnSizeChangedMethod(function<void(wxSizeEvent&)> onSizeChangedMethod)
{
	this->onSizeChangedMethod = onSizeChangedMethod;
}

void Window::onSizeChanged(wxSizeEvent& sizeEvent)
{
	if(onSizeChangedMethod != nullptr) onSizeChangedMethod(sizeEvent);
	sizeEvent.Skip();
}

wxBEGIN_EVENT_TABLE(Window, wxFrame)
EVT_CHAR_HOOK(Window::keyCharHook)
EVT_SIZE(Window::onSizeChanged)
wxEND_EVENT_TABLE()

}}
