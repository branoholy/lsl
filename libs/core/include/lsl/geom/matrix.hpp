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

#ifndef LSL_GEOM_MATRIX_HPP
#define LSL_GEOM_MATRIX_HPP

#undef NDEBUG
#include <cassert>
#include <type_traits>

#include <Eigen/Dense>

#include "lsl/utils/cpputils.hpp"
#include "lsl/utils/mathutils.hpp"

namespace lsl {
namespace geom {

template<typename _Scalar, int _Rows, int _Cols, int _Options = Eigen::ColMajor, int _MaxRows = _Rows, int _MaxCols = _Cols>
class Matrix : public Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
{
protected:
	int id;

public:
	typedef _Scalar value_type;

	typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> Base;
	EIGEN_DENSE_PUBLIC_INTERFACE(Matrix)

	bool realPoint;

	Matrix();
	Matrix(const Scalar& x, const Scalar& y);
	Matrix(const Scalar& x, const Scalar& y, const Scalar& z);
	Matrix(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w);
	Matrix(std::initializer_list<Scalar>& data);
	Matrix(const Scalar *data);
	Matrix(const Matrix& other);

	template<typename OtherDerived>
	Matrix(const Eigen::MatrixBase<OtherDerived>& other);

	inline Scalar at(std::size_t i, std::size_t j) const { return this->operator()(i,j); }
	inline Scalar& at(std::size_t i, std::size_t j) { return this->operator()(i,j); }
	inline Scalar at(std::size_t i) const { return this->operator[](i); }
	inline Scalar& at(std::size_t i) { return this->operator[](i); }

	inline int getId() const { return id; }
	inline void setId(int id) { this->id = id; }

	inline double getAngle2D() const { return utils::MathUtils::normAngle(std::atan2(at(1), at(0))); }

	template<int NewRows, int NewCols = _Cols>
	Matrix<_Scalar, NewRows, NewCols> to(Scalar defaultValue = 0) const;

	template<typename NewScalar, int NewRows = _Rows, int NewCols = _Cols>
	Matrix<NewScalar, NewRows, NewCols> to(NewScalar defaultValue = 0) const;

	inline Matrix<_Scalar, _Rows - 1, _Cols> toHeterogenous() const { return to<_Scalar, _Rows - 1, _Cols>(); }
	inline Matrix<_Scalar, _Rows + 1, _Cols> toHomogenous() const { return to<_Scalar, _Rows + 1, _Cols>(1); }

	Matrix& operator=(const Matrix& other);

	template<typename OtherDerived>
	Matrix& operator=(const Eigen::MatrixBase<OtherDerived>& other);
};

typedef Matrix<int, 2, 1> Vector2i;
typedef Matrix<int, 3, 1> Vector3i;
typedef Matrix<int, 4, 1> Vector4i;

typedef Matrix<double, 2, 1> Vector2d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 4, 1> Vector4d;

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix() : Base(),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(const Scalar& x, const Scalar& y) : Base(x, y),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(const Scalar& x, const Scalar& y, const Scalar& z) : Base(x, y, z),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& w) : Base(x, y, z, w),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(std::initializer_list<Scalar>& data) : Base(data),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix::Matrix(const Scalar *data) : Base(data),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(const Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& other) : Base(other),
	id(other.id), realPoint(other.realPoint)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
template<typename OtherDerived>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Matrix(const Eigen::MatrixBase<OtherDerived>& other) : Base(other),
	id(-1), realPoint(true)
{
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::operator=(const Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& other)
{
	this->Base::operator=(other);

	id = other.id;
	realPoint = other.realPoint;

	return *this;
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
template<typename OtherDerived>
Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::operator=(const Eigen::MatrixBase<OtherDerived>& other)
{
	this->Base::operator=(other);
	return *this;
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
template<int NewRows, int NewCols>
Matrix<_Scalar, NewRows, NewCols> Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::to(Scalar defaultValue) const
{
	return to<_Scalar, NewRows, NewCols>(defaultValue);
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
template<typename NewScalar, int NewRows, int NewCols>
Matrix<NewScalar, NewRows, NewCols> Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::to(NewScalar defaultValue) const
{
	int minRows = std::min(_Rows, NewRows);
	int minCols = std::min(_Cols, NewCols);

	auto newMatrix = Matrix<NewScalar, NewRows, NewCols>();
	newMatrix.fill(defaultValue);

	for(int r = 0; r < minRows; r++)
	{
		for(int c = 0; c < minCols; c++)
		{
			newMatrix.at(r, c) = NewScalar(at(r, c));
		}
	}

	return newMatrix;
}

}}

namespace Eigen {
namespace internal {

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct traits<lsl::geom::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> >
{
	typedef _Scalar Scalar;
	typedef Dense StorageKind;
#if EIGEN_VERSION_AT_LEAST(3,2,92)
	typedef Eigen::Index StorageIndex;
#else
	typedef DenseIndex Index;
#endif
	typedef MatrixXpr XprKind;

	enum {
		RowsAtCompileTime = _Rows,
		ColsAtCompileTime = _Cols,
		MaxRowsAtCompileTime = _MaxRows,
		MaxColsAtCompileTime = _MaxCols,
		Flags = compute_matrix_flags<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::ret,
		CoeffReadCost = NumTraits<Scalar>::ReadCost,
		Options = _Options,
		InnerStrideAtCompileTime = 1,
		OuterStrideAtCompileTime = (Options&RowMajor) ? ColsAtCompileTime : RowsAtCompileTime
	};
};

}}

#endif // LSL_GEOM_MATRIX_HPP
