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

#ifndef LSL_SYSTEM_EVENT_HPP
#define LSL_SYSTEM_EVENT_HPP

#include <algorithm>
#include <functional>
#include <vector>

namespace lsl {
namespace system {

template<typename T>
class Event;

template<typename R, typename... Args>
class Event<R(Args...)>
{
private:
	std::vector<std::function<R(Args...)>> funcs;

public:
	inline bool isEmpty() const { return funcs.empty(); }
	inline std::size_t size() const { return funcs.size(); }

	Event<R(Args...)>& operator+=(const std::function<R(Args...)>& func);
	Event<R(Args...)>& operator-=(const std::function<R(Args...)>& func);
	void operator()(Args... args);
};

template<typename R, typename... Args>
Event<R(Args...)>& Event<R(Args...)>::operator+=(const std::function<R(Args...)>& func)
{
	funcs.push_back(func);
	return *this;
}

template<typename R, typename... Args>
Event<R(Args...)>& Event<R(Args...)>::operator-=(const std::function<R(Args...)>& func)
{
	typedef R(T)(Args...);

	// FIXME: Also remove bind expressions.
	auto p = std::remove_if(funcs.begin(), funcs.end(),
		[&func](const std::function<R(Args...)>& f)
		{
			return *f.template target<T*>() == *func.template target<T*>();
		});

	funcs.erase(p, funcs.end());

	return *this;
}

template<typename R, typename... Args>
void Event<R(Args...)>::operator()(Args... args)
{
	for(auto f : funcs)
	{
		f(args...);
	}
}

}}

#endif // LSL_SYSTEM_EVENT_HPP
