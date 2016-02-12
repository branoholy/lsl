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

#include "lsl/io/carmencommand.hpp"

#include <iostream>
#include <sstream>

namespace lsl {
namespace io {

std::string CARMENCommand::CMD_PARAM = "PARAM";
std::string CARMENCommand::CMD_SYNC = "SYNC";
std::string CARMENCommand::CMD_ODOM = "ODOM";
std::string CARMENCommand::CMD_FLASER = "FLASER";
std::string CARMENCommand::CMD_RLASER = "RLASER";
std::string CARMENCommand::CMD_TRUEPOS = "TRUEPOS";

CARMENCommand::CARMENCommand() :
	command(nullptr), id(-1)
{
}

CARMENCommand::~CARMENCommand()
{
	delete command;
}

CARMENCommandParam* CARMENCommand::param() const
{
	if(name == CMD_PARAM) return static_cast<CARMENCommandParam*>(command);
	return nullptr;
}

CARMENCommandSync* CARMENCommand::sync() const
{
	if(name == CMD_SYNC) return static_cast<CARMENCommandSync*>(command);
	return nullptr;
}

CARMENCommandOdom* CARMENCommand::odom() const
{
	if(name == CMD_ODOM) return static_cast<CARMENCommandOdom*>(command);
	return nullptr;
}

CARMENCommandFLaser* CARMENCommand::flaser() const
{
	if(name == CMD_FLASER) return static_cast<CARMENCommandFLaser*>(command);
	return nullptr;
}

CARMENCommandRLaser* CARMENCommand::rlaser() const
{
	if(name == CMD_RLASER) return static_cast<CARMENCommandRLaser*>(command);
	return nullptr;
}

CARMENCommandTruepos* CARMENCommand::truepos() const
{
	if(name == CMD_TRUEPOS) return static_cast<CARMENCommandTruepos*>(command);
	return nullptr;
}

bool CARMENCommand::parse(const std::string& cmd, const std::string& onlyCmdName)
{
	if(cmd.empty()) return false;
	if(cmd[0] == '#') return false;

	std::istringstream cmdStream(cmd);
	cmdStream >> name;

	if(!onlyCmdName.empty() && name != onlyCmdName) return false;

	if(name == CMD_PARAM)
	{
		CARMENCommandParam *commandParam = new CARMENCommandParam;
		cmdStream >> commandParam->name >> commandParam->value;

		command = commandParam;
	}
	else if(name == CMD_SYNC)
	{
		CARMENCommandSync *commandSync = new CARMENCommandSync;
		cmdStream >> commandSync->tagName;

		command = commandSync;
	}
	else if(name == CMD_ODOM)
	{
		CARMENCommandOdom *commandOdom = new CARMENCommandOdom;
		cmdStream >> commandOdom->x >> commandOdom->y >> commandOdom->theta
				>> commandOdom->tv >> commandOdom->rv >> commandOdom->accel;

		command = commandOdom;
	}
	else if(name == CMD_FLASER || name == CMD_RLASER)
	{
		std::size_t numReadings;
		cmdStream >> numReadings;

		double *rangeReadings = new double[numReadings];
		for(std::size_t i = 0; i < numReadings; i++)
		{
			cmdStream >> rangeReadings[i];
		}

		double x, y, theta, odomX, odomY, odomTheta;
		cmdStream >> x >> y >> theta >> odomX >> odomY >> odomTheta;

		if(name == CMD_FLASER)
		{
			CARMENCommandFLaser *commandFLaser = new CARMENCommandFLaser;
			commandFLaser->numReadings = numReadings;
			commandFLaser->rangeReadings = rangeReadings;
			commandFLaser->x = x;
			commandFLaser->y = y;
			commandFLaser->theta = theta;
			commandFLaser->odomX = odomX;
			commandFLaser->odomY = odomY;
			commandFLaser->odomTheta = odomTheta;

			command = commandFLaser;
		}
		else
		{
			CARMENCommandRLaser *commandRLaser = new CARMENCommandRLaser;
			commandRLaser->numReadings = numReadings;
			commandRLaser->rangeReadings = rangeReadings;
			commandRLaser->x = x;
			commandRLaser->y = y;
			commandRLaser->theta = theta;
			commandRLaser->odomX = odomX;
			commandRLaser->odomY = odomY;
			commandRLaser->odomTheta = odomTheta;

			command = commandRLaser;
		}
	}
	else if(name == CMD_TRUEPOS)
	{
		CARMENCommandTruepos *commandTruepos = new CARMENCommandTruepos;
		cmdStream >> commandTruepos->trueX >> commandTruepos->trueY >> commandTruepos->trueTheta
				  >> commandTruepos->odomX >> commandTruepos->odomY >> commandTruepos->odomTheta;

		command = commandTruepos;
	}
	else
	{
		return false;
	}

	cmdStream >> ipcTimestamp >> ipcHostname >> loggerTimestamp;
	if(cmdStream) cmdStream >> id;

	return true;
}

std::ostream& operator<<(std::ostream& out, const CARMENCommand& command)
{
	out << command.name << ' ' << *command.command << ' ' << command.ipcTimestamp << ' ' << command.ipcHostname << ' ' << command.loggerTimestamp;
	if(command.id != std::size_t(-1)) out << command.id;

	return out;
}

CARMENCommandType::~CARMENCommandType()
{
}

std::ostream& CARMENCommandType::print(std::ostream& out) const
{
	return out;
}

std::ostream& operator<<(std::ostream& out, const CARMENCommandType& command)
{
	return command.print(out);
}

std::ostream& CARMENCommandParam::print(std::ostream& out) const
{
	return out << name << ' ' << value;
}

std::ostream& CARMENCommandSync::print(std::ostream& out) const
{
	return out << tagName;
}

std::ostream& CARMENCommandOdom::print(std::ostream& out) const
{
	return out << x << ' ' << y << ' ' << theta << ' ' << tv << ' ' << rv << ' ' << accel;
}

CARMENCommandFLaser::CARMENCommandFLaser() :
	rangeReadings(nullptr)
{
}

CARMENCommandFLaser::~CARMENCommandFLaser()
{
	delete[] rangeReadings;
}

std::ostream& CARMENCommandFLaser::print(std::ostream& out) const
{
	out << numReadings << ' ';
	for(std::size_t i = 0; i < numReadings; i++) out << rangeReadings[i] << ' ';
	out << x << ' ' << y << ' ' << theta << ' ' << odomX << ' ' << odomY << ' ' << odomTheta;

	return out;
}

CARMENCommandRLaser::CARMENCommandRLaser() : CARMENCommandFLaser()
{
}

std::ostream& CARMENCommandTruepos::print(std::ostream& out) const
{
	return out << trueX << ' ' << trueY << ' ' << trueTheta << ' ' << odomX << ' ' << odomY << ' ' << odomTheta;
}

}}
