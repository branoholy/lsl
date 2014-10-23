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

#ifndef LSL_GUI_REPAINTINGAPP_HPP
#define LSL_GUI_REPAINTINGAPP_HPP

#include <string>
#include <functional>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "window.hpp"

namespace lsl {
namespace gui {

class LSLApp : public wxApp
{
private:
	std::string title;
	wxSize windowSize;
	Window *window;

	std::function<void(Window*)> onInitMethod;

public:
	LSLApp(const std::string& title = "LSL App", const wxSize& windowSize = wxDefaultSize);
	virtual bool OnInit();

	inline void setOnInitMethod(std::function<void(Window*)> onInitMethod) { this->onInitMethod = onInitMethod; }

	static void Display(LSLApp *app, int& argc, char **argv);
};

}}

#endif // LSL_GUI_REPAINTINGAPP_HPP
