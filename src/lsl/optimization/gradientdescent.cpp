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

#include "lsl/optimization/gradientdescent.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

#include "lsl/utils/arrayutils.hpp"

#define LSL_OPTIMIZATION_GD_PRINT

using namespace std;

using namespace lsl::utils;

namespace lsl {
namespace optimization {

GradientDescent::GradientDescent(size_t dimension, double* deltas, double* gammas, double precision) :
	dimension(dimension), deltas(deltas), gammas(gammas), precision(precision), evals(0), converged(false)
{
}

double* GradientDescent::minimize(function<double(const double*)> errorFunction, const double *guess, double& output)
{
	evals = 0;
	converged = false;

	size_t size = dimension * sizeof(double);
	double *input = new double[dimension];
	double *newInput = new double[dimension];
	double *gradient = new double[dimension];
	double *gammas = new double[dimension];

	memcpy(input, guess, size);
	memcpy(newInput, guess, size);
	memcpy(gammas, this->gammas, size);

	output = errorFunction(input);
	evals++;

	for(size_t k = 0; k < 10000; k++)
	{

#ifdef LSL_OPTIMIZATION_GD_PRINT
		cout << "Actual: " << output << " = f(";
		ArrayUtils::printArray(cout, input, dimension);
		cout << ')' << endl << endl << "For Gradient:" << endl;
#endif

		getGradient(errorFunction, newInput, output, gradient);

#ifdef LSL_OPTIMIZATION_GD_PRINT
		cout << endl << "Gradient: (";
		ArrayUtils::printArray(cout, gradient, dimension);
		cout << ')' << endl;
#endif

		for(size_t i = 0; i < dimension; i++)
		{
			newInput[i] -= gammas[i] * gradient[i];
		}

		double newOutput = errorFunction(newInput);
		evals++;

#ifdef LSL_OPTIMIZATION_GD_PRINT
		cout << "New: " << newOutput << " = f(";
		ArrayUtils::printArray(cout, newInput, dimension);
		cout << ')' << endl;

		cout << "-----------" << endl;
#endif

		if(inputDistance(input, newInput) <= precision)
		{
			converged = true;
			break;
		}
		else
		{
			if(newOutput > output)
			{
				cout << "G " << output << ' ' << newOutput << endl;
				for(size_t i = 0; i < dimension; i++)
				{
					gammas[i] *= 0.9;
				}
			}

			output = newOutput;
			memcpy(input, newInput, size);
		}
	}

	delete[] newInput;
	delete[] gradient;
	delete[] gammas;

	return input;
}

double* GradientDescent::minimizeByDimension(function<double(const double*)> errorFunction, const double *guess, double& output)
{
	size_t size = dimension * sizeof(double);
	double *input = new double[dimension];
	double *newInput = new double[dimension];
	double *gradient = new double[dimension];

	memcpy(input, guess, size);
	memcpy(newInput, guess, size);
	output = errorFunction(input);

	while(true)
	{
		double minOutput = output;
		size_t minDimension = dimension + 1;
		for(size_t i = 0; i < dimension; i++)
		{
			newInput[i] += deltas[i];
			double deltaOutput = errorFunction(newInput);
			if(deltaOutput < minOutput)
			{
				minOutput = deltaOutput;
				minDimension = i;
			}

			newInput[i] -= deltas[i];
		}

		// Found something smaller
		if(minDimension < dimension)
		{
			input[minDimension] += deltas[minDimension];
			output = minOutput;
		}
		else
		{
			break;
		}
	}
	delete[] gradient;
	delete[] newInput;

	return input;
}

double GradientDescent::inputDistance(const double *input1, const double *input2) const
{
	double distance = 0;
	for(size_t i = 0; i < dimension; i++)
	{
		distance += pow(input1[i] - input2[i], 2);
	}

	return sqrt(distance);
}

void GradientDescent::getGradient(const function<double(const double *)>& errorFunction, double *input, double output, double *gradient)
{
	for(size_t i = 0; i < dimension; i++)
	{
		input[i] += deltas[i];
		double deltaOutput = errorFunction(input);
		evals++;

#ifdef LSL_OPTIMIZATION_GD_PRINT
		cout << deltaOutput << " = ";
		ArrayUtils::printArray(cout, input, dimension);
		cout << endl;
#endif

		gradient[i] = (deltaOutput - output) / deltas[i];
		input[i] -= deltas[i];
	}
}

}}
