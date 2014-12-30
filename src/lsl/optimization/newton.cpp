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

#include "lsl/optimization/newton.hpp"

#include <cstring>
#include <iostream>

#include "lsl/utils/arrayutils.hpp"

#define LSL_OPTIMIZATION_NEWTON_PRINT

using namespace std;

using namespace lsl::utils;
using namespace lsl::optimization;

Newton::Newton(size_t dimension, double *deltas, double *gammas, double precision) :
	dimension(dimension), deltas(deltas), gammas(gammas), precision(precision), evals(0), converged(false)
{
}

double* Newton::minimize(function<double(const double*)> errorFunction, const double *guess, double& output)
{
	evals = 0;
	converged = false;

	size_t size = dimension * sizeof(double);
	double *input = new double[dimension];
	double *newInput = new double[dimension];
	double *gammas = new double[dimension];

	memcpy(input, guess, size);
	memcpy(newInput, guess, size);
	memcpy(gammas, this->gammas, size);

	output = errorFunction(input);
	evals++;

	for(size_t k = 0; k < 10000; k++)
	{
		Eigen::VectorXd gradient(dimension);
		Eigen::MatrixXd hessian(dimension, dimension);

#ifdef LSL_OPTIMIZATION_NEWTON_PRINT
		cout << "Old: " << output << " = ";
		ArrayUtils::printArray(cout, input, dimension);
		cout << endl;
#endif

		getGradientAndHessian(errorFunction, newInput, output, gradient, hessian);

		Eigen::VectorXd deltaInput = hessian.inverse() * gradient;

#ifdef LSL_OPTIMIZATION_NEWTON_PRINT
		cout << "Gradient: " << gradient.transpose() << endl;
		cout << "Hessian: " << endl << hessian << endl;
		cout << "Delta: " << deltaInput.transpose() << endl;
#endif

		for(size_t i = 0; i < dimension; i++)
		{
			newInput[i] -= gammas[i] * deltaInput(i);
		}

		double newOutput = errorFunction(newInput);
		evals++;

#ifdef LSL_OPTIMIZATION_NEWTON_PRINT
		cout << "New: " << newOutput << " = ";
		ArrayUtils::printArray(cout, input, dimension);
		cout << endl;
#endif

		/*
		if(newOutput > output)
		{
			converged = true;
			break;
		}
		else
		{
			output = newOutput;
			memcpy(input, newInput, size);
		}
		*/

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

	return input;
}

double Newton::inputDistance(const double *input1, const double *input2) const
{
	double distance = 0;
	for(size_t i = 0; i < dimension; i++)
	{
		distance += pow(input1[i] - input2[i], 2);
	}

	return sqrt(distance);
}

void Newton::getGradientAndHessian(function<double(const double*)> f, double *input, double output, Eigen::VectorXd& gradient, Eigen::MatrixXd& hessian)
{
	double *fi = new double[dimension];

	for(size_t i = 0; i < dimension; i++)
	{
		input[i] += deltas[i];
		fi[i] = f(input);
		evals++;
		input[i] -= deltas[i];

		gradient(i) = (fi[i] - output) / deltas[i];
	}

	for(size_t i = 0; i < dimension; i++)
	{
		input[i] += deltas[i];
		for(size_t j = 0; j < dimension; j++)
		{
			input[j] += deltas[j];
			hessian(i, j) = (((f(input) - fi[j]) / deltas[i]) - gradient(i)) / deltas[j];
			evals++;
			input[j] -= deltas[j];
		}
		input[i] -= deltas[i];
	}

	delete[] fi;
}
