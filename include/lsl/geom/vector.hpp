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

#ifndef LSL_GEOM_VECTOR_HPP
#define LSL_GEOM_VECTOR_HPP

#include <cmath>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <vector>

#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace geom {

template<typename T, unsigned int dim>
class Vector
{
protected:
	T data[dim];
	unsigned int id;

public:
	Vector();
	Vector(unsigned int id);
	Vector(const T *data, std::size_t dataSize);
	Vector(std::initializer_list<T> data);
	Vector(const std::vector<T>& data);
	Vector(const Vector<T, dim>& vector);

	typedef T value_type;
	static unsigned int dimension_size;

	inline unsigned int getDim() const	{ return dim; }

	void set(const T *data, std::size_t dataSize);
	void set(std::initializer_list<T> data);
	void set(const std::vector<T>& data);
	void set(const Vector<T, dim>& vector);

	inline int getId() const { return id; }
	inline void setId(unsigned int id) { this->id = id; }

	double getLength() const;
	double getLength2() const;
	double getLength2D2(unsigned int dim1, unsigned int dim2) const;
	double getLength2D(unsigned int dim1, unsigned int dim2) const;

	void setLength(double length);
	void normalize();

	double getDistanceTo(const Vector<T, dim>& other) const;
	double getDistanceTo2(const Vector<T, dim>& other) const;

	double getAngle2D() const;
	void setAngle2D(double angle);

	double cosine(const Vector<T, dim>& other) const;

	void rotate2D(double angle);
	void rotate2D(double c, double s);
	void transform2D(double angle, double tx, double ty);
	void transform2D(double c, double s, double tx, double ty);

	bool operator<(const Vector<T, dim>& other) const;
	bool operator==(const Vector<T, dim>& other) const;

	Vector<T, dim> operator+(const Vector<T, dim>& other) const;
	Vector<T, dim>& operator+=(const Vector<T, dim>& other);

	Vector<T, dim> operator-(const Vector<T, dim>& other) const;
	Vector<T, dim>& operator-=(const Vector<T, dim>& other);

	Vector<T, dim> operator*(const double m) const;
	Vector<T, dim>& operator*=(const double m);

	Vector<T, dim> operator/(const double m) const;
	Vector<T, dim>& operator/=(const double m);

	double operator*(const Vector<T, dim>& other) const;

	T& operator[](unsigned int i);
	const T& operator[](unsigned int i) const;

	static Vector<T, dim> getCentroid(const std::vector<Vector<T, dim>>& points);
	static Vector<T, dim> getCentroid(const std::vector<Vector<T, dim>*>& points);

	template<typename T_, unsigned int dim_>
	friend std::ostream& operator<<(std::ostream& out, const Vector<T_, dim_>& vector);

	template<typename T_, unsigned int dim_>
	friend std::ofstream& operator<<(std::ofstream& out, const Vector<T_, dim_>& vector);
};

typedef Vector<double, 2> Vector2d;
typedef Vector<double, 3> Vector3d;
typedef Vector<double, 4> Vector4d;

typedef Vector<int, 2> Vector2i;
typedef Vector<int, 3> Vector3i;
typedef Vector<int, 4> Vector4i;

template<typename T, unsigned int dim>
unsigned int Vector<T, dim>::dimension_size = dim;

template<typename T, unsigned int dim>
Vector<T, dim>::Vector() : Vector(-1)
{
}

template<typename T, unsigned int dim>
Vector<T, dim>::Vector(unsigned int id) :
	id(id)
{
}

template<typename T, unsigned int dim>
Vector<T, dim>::Vector(const T *data, std::size_t dataSize) : Vector()
{
	set(data, dataSize);
}

template<typename T, unsigned int dim>
Vector<T, dim>::Vector(std::initializer_list<T> data) : Vector()
{
	set(data);
}

template<typename T, unsigned int dim>
Vector<T, dim>::Vector(const std::vector<T>& data) : Vector()
{
	set(data);
}

template<typename T, unsigned int dim>
Vector<T, dim>::Vector(const Vector<T, dim>& vector) : Vector()
{
	set(vector);
}

template<typename T, unsigned int dim>
void Vector<T, dim>::set(const T *data, std::size_t dataSize)
{
	if(dim < dataSize)
	{
		id = data[0];
		memcpy(this->data, data + 1, sizeof(T) * dim);
	}
	else
	{
		memcpy(this->data, data, sizeof(T) * dim);
	}
}

template<typename T, unsigned int dim>
void Vector<T, dim>::set(std::initializer_list<T> data)
{
	typename std::initializer_list<T>::iterator it = data.begin();
	if(dim < data.size()) id = *(it++);

	for(unsigned int i = 0; i < dim; i++, it++)
	{
		this->data[i] = *it;
	}
}

template<typename T, unsigned int dim>
void Vector<T, dim>::set(const std::vector<T>& data)
{
	typename std::vector<T>::iterator it = data.begin();
	if(dim < data.size()) id = *(it++);

	for(unsigned int i = 0; i < dim; i++, it++)
	{
		this->data[i] = *it;
	}
}

template<typename T, unsigned int dim>
void Vector<T, dim>::set(const Vector<T, dim>& vector)
{
	for(unsigned int i = 0; i < dim; i++)
	{
		data[i] = vector[i];
	}
	id = vector.getId();
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getLength() const
{
	return std::sqrt(getLength2());
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getLength2() const
{
	T len2 = 0;
	for(unsigned int i = 0; i < dim; i++)
	{
		len2 += std::pow(data[i], 2);
	}

	return len2;
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getLength2D(unsigned int dim1, unsigned int dim2) const
{
	return std::sqrt(getLength2D2(dim1, dim2));
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getLength2D2(unsigned int dim1, unsigned int dim2) const
{
	return std::pow(data[dim1], 2) + std::pow(data[dim2], 2);
}

template<typename T, unsigned int dim>
void Vector<T, dim>::setLength(double length)
{
	double mul = length / getLength();
	operator*=(mul);
}

template<typename T, unsigned int dim>
void Vector<T, dim>::normalize()
{
	setLength(1);
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getDistanceTo(const Vector<T, dim>& other) const
{
	return std::sqrt(getDistanceTo2(other));
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getDistanceTo2(const Vector<T, dim>& other) const
{
	T distance2 = 0;
	for(unsigned int i = 0; i < dim; i++)
	{
		distance2 += std::pow(data[i] - other[i], 2);
	}

	return distance2;
}

template<typename T, unsigned int dim>
double Vector<T, dim>::getAngle2D() const
{
	return utils::MathUtils::normAngle(atan2(data[1], data[0]));
}

template<typename T, unsigned int dim>
void Vector<T, dim>::setAngle2D(double angle)
{
	double length = getLength2D(0, 1);

	data[0] = std::cos(angle) * length;
	data[1] = std::sin(angle) * length;
}

template<typename T, unsigned int dim>
double Vector<T, dim>::cosine(const Vector<T, dim>& other) const
{
	return ((*this) * other) / (this->getLength() * other.getLength());
}

template<typename T, unsigned int dim>
void Vector<T, dim>::rotate2D(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	rotate2D(c, s);
}

template<typename T, unsigned int dim>
void Vector<T, dim>::rotate2D(double c, double s)
{
	double x = data[0] * c - data[1] * s;
	double y = data[0] * s + data[1] * c;
	data[0] = x;
	data[1] = y;
}

template<typename T, unsigned int dim>
void Vector<T, dim>::transform2D(double angle, double tx, double ty)
{
	double c = cos(angle);
	double s = sin(angle);

	transform2D(c, s, tx, ty);
}

template<typename T, unsigned int dim>
void Vector<T, dim>::transform2D(double c, double s, double tx, double ty)
{
	rotate2D(c, s);
	data[0] += tx;
	data[1] += ty;
}

template<typename T, unsigned int dim>
bool Vector<T, dim>::operator<(const Vector<T, dim>& other) const
{
	return getLength2() < other.getLength2();
}

template<typename T, unsigned int dim>
bool Vector<T, dim>::operator==(const Vector<T, dim>& other) const
{
	for(unsigned int i = 0; i < dim; i++)
	{
		if(data[i] != other[i])
		{
			return false;
		}
	}

	return true;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::operator+(const Vector<T, dim>& other) const
{
	Vector<T, dim> r(*this);
	r += other;
	return r;
}

template<typename T, unsigned int dim>
Vector<T, dim>& Vector<T, dim>::operator+=(const Vector<T, dim>& other)
{
	for(unsigned int i = 0; i < dim; i++)
	{
		data[i] += other[i];
	}

	return *this;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::operator-(const Vector<T, dim>& other) const
{
	Vector<T, dim> r(*this);
	r -= other;
	return r;
}

template<typename T, unsigned int dim>
Vector<T, dim>& Vector<T, dim>::operator-=(const Vector<T, dim>& other)
{
	for(unsigned int i = 0; i < dim; i++)
	{
		data[i] -= other[i];
	}

	return *this;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::operator*(const double mul) const
{
	Vector<T, dim> r(*this);
	r *= mul;
	return r;
}

template<typename T, unsigned int dim>
Vector<T, dim>& Vector<T, dim>::operator*=(const double mul)
{
	for(unsigned int i = 0; i < dim; i++)
	{
		data[i] *= mul;
	}

	return *this;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::operator/(const double mul) const
{
	Vector<T, dim> r(*this);
	r /= mul;
	return r;
}

template<typename T, unsigned int dim>
Vector<T, dim>& Vector<T, dim>::operator/=(const double mul)
{
	for(unsigned int i = 0; i < dim; i++)
	{
		data[i] /= mul;
	}

	return *this;
}

template<typename T, unsigned int dim>
double Vector<T, dim>::operator*(const Vector<T, dim>& other) const
{
	double dotProduct = 0;
	for(unsigned int i = 0; i < dim; i++)
	{
		dotProduct += data[i] * other[i];
	}

	return dotProduct;
}

template<typename T, unsigned int dim>
T& Vector<T, dim>::operator[](unsigned int i)
{
	return data[i];
}

template<typename T, unsigned int dim>
const T& Vector<T, dim>::operator[](unsigned int i) const
{
	return data[i];
}

template<typename T, unsigned int dim>
Vector<T, dim> operator*(const double mul, const Vector<T, dim>& v)
{
	Vector<T, dim> r(v);
	r *= mul;
	return r;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::getCentroid(const std::vector<Vector<T, dim>>& points)
{
	Vector<T, dim> centroid;
	for(std::size_t i = 0; i < points.size(); i++)
	{
		centroid += points.at(i);
	}
	centroid *= 1.0 / points.size();

	return centroid;
}

template<typename T, unsigned int dim>
Vector<T, dim> Vector<T, dim>::getCentroid(const std::vector<Vector<T, dim>*>& points)
{
	Vector<T, dim> centroid;
	for(std::size_t i = 0; i < points.size(); i++)
	{
		centroid += *points.at(i);
	}
	centroid *= 1.0 / points.size();

	return centroid;
}

template<typename T, unsigned int dim>
std::ostream& operator<<(std::ostream& out, const Vector<T, dim>& vector)
{
	out << "V:";
	if(vector.id != -1) out << vector.id;

	out << '[' << vector[0];
	for(unsigned int i = 1; i < dim; i++)
	{
		out << ", " << vector[i];
	}
	out << ']';

	return out;
}

template<typename T, unsigned int dim>
std::ofstream& operator<<(std::ofstream& out, const Vector<T, dim>& vector)
{
	out << vector[0];
	for(unsigned int i = 1; i < dim; i++)
	{
		out << ' ' << vector[i];
	}

	return out;
}

}}

#endif // LSL_GEOM_VECTOR_HPP
