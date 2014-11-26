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

#include "lsl/optimization/soma.hpp"

#include <random>
#include <cstring>

#include "lsl/utils/arrayutils.hpp"

using namespace std;
using namespace lsl::utils;

namespace lsl {
namespace optimization {

SOMA::SOMA(size_t dimension, double mass, double step, double prt, size_t np, size_t migration, double acceptedError) :
	dimension(dimension), oi(dimension + 1), mass(mass), step(step), prt(prt), np(np), migration(migration), acceptedError(acceptedError), evals(0), converged(false)
{
	specimen = new Speciman[dimension];
	population = ArrayUtils::create2dArray<double>(np, oi);
}

SOMA::~SOMA()
{
	delete[] specimen;
	ArrayUtils::delete2dArray(population, np);
}

void SOMA::setSpecimen(size_t dimension, double low, double high)
{
	specimen[dimension].low = low;
	specimen[dimension].high = high;
}

void SOMA::setSpecimen(const double specimen[][2])
{
	for(size_t i = 0; i < dimension; i++)
	{
		setSpecimen(i, specimen[i][0], specimen[i][1]);
	}
}

void SOMA::initUniformPopulation()
{
	random_device rd;
	mt19937 rng(rd());
	uniform_real_distribution<> *dists = new uniform_real_distribution<>[dimension];
	for(size_t i = 0; i < dimension; i++)
	{
		dists[i] = uniform_real_distribution<>(specimen[i].low, specimen[i].high);
	}

	for(size_t i = 0; i < np; i++)
	{
		double *individual = population[i];
		for(size_t j = 0; j < dimension; j++)
		{
			individual[j] = dists[j](rng);
		}
	}

	delete[] dists;
}

void SOMA::initNormalPopulation(const double *mean, const double *sigmas)
{
	random_device rd;
	mt19937 rng(rd());
	normal_distribution<> *dists = new normal_distribution<>[dimension];
	for(size_t i = 0; i < dimension; i++)
	{
		dists[i] = normal_distribution<>(mean[i], sigmas[i]);
	}

	for(size_t i = 0; i < np; i++)
	{
		double *individual = population[i];
		for(size_t j = 0; j < dimension; j++)
		{
			double value;

			do
			{
				value = dists[j](rng);
			}
			while(value < specimen[j].low || value > specimen[j].high);

			individual[j] = value;
		}
	}

	delete[] dists;
}

double* SOMA::minimize(function<double(const double*)> errorFunction, double& output)
{
	evals = 0;
	converged = false;

	size_t indSize = sizeof(double) * dimension;
	bool *prtVector = new bool[dimension];
	double *tmpInd = new double[dimension];
	double *bestInd = new double[dimension];

	random_device rd;
	mt19937 rng(rd());
	uniform_real_distribution<> dist(0, 1);

	for(size_t i = 0; i < np; i++)
	{
		population[i][oi] = errorFunction(population[i]);
		evals++;
	}

	double *leader;
	for(size_t lap = 0; ; lap++)
	{
		size_t leaderIndex = 0;
		double leaderOutput, worstOutput;
		leaderOutput = worstOutput = population[0][oi];
		for(size_t i = 1; i < np; i++)
		{
			double output = population[i][oi];
			if(output < leaderOutput)
			{
				leaderIndex = i;
				leaderOutput = output;
			}
			if(output > worstOutput)
			{
				worstOutput = output;
			}
		}
		leader = population[leaderIndex];
		// cout << lap << ' ' << leaderOutput << endl;

		if(abs(leaderOutput - worstOutput) <= acceptedError)
		{
			converged = true;
			break;
		}

		if(lap >= migration) break;

		for(size_t i = 0; i < np; i++)
		{
			if(i == leaderIndex) continue;

			for(size_t j = 0; j < dimension; j++)
			{
				prtVector[j] = (dist(rng) < prt);
			}

			double *individual = population[i];
			double bestOutput = individual[oi];
			for(double t = 0; t < mass; t += step)
			{
				for(size_t j = 0; j < dimension; j++)
				{
					tmpInd[j] = individual[j] + (leader[j] - individual[j]) * t * prtVector[j];
					if(tmpInd[j] < specimen[j].low || tmpInd[j] > specimen[j].high)
					{
						tmpInd[j] = specimen[j].low + dist(rng) * (specimen[j].high - specimen[j].low);
					}
				}
				double output = errorFunction(tmpInd);
				evals++;

				if(output < bestOutput)
				{
					memcpy(bestInd, tmpInd, indSize);
					bestOutput = output;
				}
			}

			memcpy(individual, bestInd, indSize);
			individual[oi] = bestOutput;
		}
	}

	delete[] prtVector;
	delete[] tmpInd;
	delete[] bestInd;

	double *r = new double[dimension];
	memcpy(r, leader, indSize);
	output = leader[oi];

	return r;
}

}}
