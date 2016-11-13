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

#ifndef LSL_OPTIMIZATION_GRADIENTDESCENT_HPP
#define LSL_OPTIMIZATION_GRADIENTDESCENT_HPP

#include <functional>

#include "lsl/geom/matrix.hpp"

namespace lsl {
namespace optimization {

template<typename Input>
class GradientDescentBase
{
protected:
	Input deltas;
	Input gammas;
	double precision;

	std::size_t evals;
	bool converged;

	virtual void fillGradient(const std::function<double(const Input&)>& errorFunction, Input input, double output, Input& gradient) = 0;
	virtual void fillInput(const Input& oldInput, const Input& gradient, Input& newInput) = 0;

public:
	GradientDescentBase(const Input& deltas, const Input& gammas, double precision);
	virtual ~GradientDescentBase() = default;

	inline std::size_t getEvaluationCount() const { return evals; }
	inline bool hasConverged() const { return converged; }

	virtual inline Input minimize(const std::function<double(const Input&)>& errorFunction, const Input& guess) final
	{
		double output;
		return minimize(errorFunction, guess, output);
	}

	virtual Input minimize(const std::function<double(const Input&)>& errorFunction, const Input& guess, double& output);
};

template<int Dimension>
class GradientDescent : public GradientDescentBase<geom::VectorNd<Dimension>>
{
protected:
	virtual void fillGradient(const std::function<double(const geom::VectorNd<Dimension>&)>& errorFunction, geom::VectorNd<Dimension> input, double output, geom::VectorNd<Dimension>& gradient) override;
	virtual void fillInput(const geom::VectorNd<Dimension>& oldInput, const geom::VectorNd<Dimension>& gradient, geom::VectorNd<Dimension>& newInput) override;

public:
	using GradientDescentBase<geom::VectorNd<Dimension>>::GradientDescentBase;
	virtual ~GradientDescent() = default;
};

template<>
class GradientDescent<1> : public GradientDescentBase<double>
{
protected:
	virtual void fillGradient(const std::function<double(const double&)>& errorFunction, double input, double output, double& gradient) override;
	virtual void fillInput(const double& oldInput, const double& gradient, double& newInput) override;

public:
	using GradientDescentBase<double>::GradientDescentBase;
	virtual ~GradientDescent() = default;
};

template<typename Input>
GradientDescentBase<Input>::GradientDescentBase(const Input& deltas, const Input& gammas, double precision) :
	deltas(deltas), gammas(gammas), precision(precision)
{
}

template<typename Input>
Input GradientDescentBase<Input>::minimize(const std::function<double(const Input&)>& errorFunction, const Input& guess, double& output)
{
	evals = 0;
	converged = false;

	Input input = guess;
	Input newInput = guess;
	Input gradient;

	output = errorFunction(input);
	evals++;

	for(std::size_t k = 0; k < 100; k++)
	{
		fillGradient(errorFunction, newInput, output, gradient);
		fillInput(input, gradient, newInput);

		double newOutput = errorFunction(newInput);
		evals++;

		output = newOutput;
		input = newInput;
	}

	return input;
}

template<int Dimension>
void GradientDescent<Dimension>::fillGradient(const std::function<double(const geom::VectorNd<Dimension>&)>& errorFunction, geom::VectorNd<Dimension> input, double output, geom::VectorNd<Dimension>& gradient)
{
	for(int i = 0; i < Dimension; i++)
	{
		input[i] += this->deltas[i];
		double deltaOutput = errorFunction(input);
		this->evals++;

		gradient[i] = (deltaOutput - output) / this->deltas[i];
		input[i] -= this->deltas[i];
	}
}

template<int Dimension>
void GradientDescent<Dimension>::fillInput(const geom::VectorNd<Dimension>& oldInput, const geom::VectorNd<Dimension>& gradient, geom::VectorNd<Dimension>& newInput)
{
	for(int d = 0; d < Dimension; d++)
	{
		newInput[d] = oldInput[d] - this->gammas[d] * gradient[d];
	}
}

void GradientDescent<1>::fillGradient(const std::function<double(const double&)>& errorFunction, double input, double output, double& gradient)
{
	this->evals++;
	gradient = (errorFunction(input + this->deltas) - output) / this->deltas;
}

void GradientDescent<1>::fillInput(const double& oldInput, const double& gradient, double& newInput)
{
	newInput = oldInput - this->gammas * gradient;
}

}}

#endif // LSL_OPTIMIZATION_GRADIENTDESCENT_HPP
