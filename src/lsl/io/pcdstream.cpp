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

#include "lsl/io/pcdstream.hpp"

#include <cctype>
#include <fstream>
#include <sstream>

using namespace std;
using namespace lsl::geom;

namespace lsl {
namespace io {

vector<Vector2d> PCDStream::load2d(const string& fileName)
{
	vector<Vector2d> points;
	ifstream file(fileName);

	int i = 0;
	string line;
	while(getline(file, line))
	{
		if(line.empty()) continue;

		char firstChar = line.at(0);
		if(firstChar == '#') continue;
		if(isalpha(firstChar)) continue;

		double x, y;
		istringstream iss(line);
		iss >> x >> y;

		points.push_back({double(i++), x, y});
	}

	return points;
}

}}
