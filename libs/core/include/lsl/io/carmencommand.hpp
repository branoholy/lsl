/*
 * LIDAR System Library
 * Copyright (C) 2014-2016  Branislav Holý <branoholy@gmail.com>
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

#ifndef LSL_IO_CARMENCOMMAND_HPP
#define LSL_IO_CARMENCOMMAND_HPP

#include <string>

namespace lsl {
namespace io {

/*
 * message_name [message contents] ipc_timestamp ipc_hostname logger_timestamp [id]
 * message formats defined: PARAM SYNC ODOM FLASER RLASER TRUEPOS
 * PARAM param_name param_value
 * SYNC tagname
 * ODOM x y theta tv rv accel
 * FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
 * RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
 * TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta
 */

class CARMENCommandType;
class CARMENCommandParam;
class CARMENCommandSync;
class CARMENCommandOdom;
class CARMENCommandFLaser;
class CARMENCommandRLaser;
class CARMENCommandTruepos;

class CARMENCommand
{
public:
	static std::string CMD_PARAM, CMD_SYNC, CMD_ODOM, CMD_FLASER, CMD_RLASER, CMD_TRUEPOS;

	std::string name;
	CARMENCommandType *command;
	double ipcTimestamp;
	std::string ipcHostname;
	double loggerTimestamp;
	std::size_t id;

	CARMENCommand();
	~CARMENCommand();

	CARMENCommandParam* param() const;
	CARMENCommandSync* sync() const;
	CARMENCommandOdom* odom() const;
	CARMENCommandFLaser* flaser() const;
	CARMENCommandRLaser* rlaser() const;
	CARMENCommandTruepos* truepos() const;

	bool parse(const std::string& cmd, const std::string& onlyCmdName = "");

	friend std::ostream& operator<<(std::ostream& out, const CARMENCommand& command);
};

class CARMENCommandType
{
public:
	virtual ~CARMENCommandType();

	virtual std::ostream& print(std::ostream& out) const;

	friend std::ostream& operator<<(std::ostream& out, const CARMENCommandType& command);
};

class CARMENCommandParam : public CARMENCommandType
{
public:
	std::string name, value;

	std::ostream& print(std::ostream& out) const;
};

class CARMENCommandSync : public CARMENCommandType
{
public:
	std::string tagName;

	std::ostream& print(std::ostream& out) const;
};

class CARMENCommandOdom : public CARMENCommandType
{
public:
	double x, y, theta, tv, rv, accel;

	std::ostream& print(std::ostream& out) const;
};

class CARMENCommandFLaser : public CARMENCommandType
{
public:
	std::size_t numReadings;
	double *rangeReadings;
	double x, y, theta, odomX, odomY, odomTheta;

	CARMENCommandFLaser();
	virtual ~CARMENCommandFLaser();

	std::ostream& print(std::ostream& out) const;
};

class CARMENCommandRLaser : public CARMENCommandFLaser
{
public:
	CARMENCommandRLaser();
};

class CARMENCommandTruepos : public CARMENCommandType
{
public:
	double trueX, trueY, trueTheta, odomX, odomY, odomTheta;

	std::ostream& print(std::ostream& out) const;
};

}}

#endif // LSL_IO_CARMENCOMMAND_HPP
