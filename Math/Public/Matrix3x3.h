#pragma once

#include "MathUtil.h"

namespace HM
{
	struct Matrix3x3
	{
	public:
		Vector3 Column[3] = { Vector3::UnitX, Vector3::UnitY, Vector3::UnitZ };

		FORCEINLINE constexpr Matrix3x3() = default;
		FORCEINLINE constexpr Matrix3x3(float _00, float _01, float _02, float _10, float _11, float _12, float _20, float _21, float _22)
		{
			Column[0].X = _00;
			Column[0].Y = _10;
			Column[0].Z = _20;
			Column[1].X = _01;
			Column[1].Y = _11;
			Column[1].Z = _21;
			Column[2].X = _02;
			Column[2].Y = _12;
			Column[2].Z = _22;
		}
		FORCEINLINE constexpr Matrix3x3(const Vector3& v1, const Vector3& v2, const Vector3& v3)
		{
			Column[0] = v1;
			Column[1] = v2;
			Column[2] = v3;
		}

		FORCEINLINE constexpr Matrix3x3 GetTranspose() const
		{
			return Matrix3x3(
				Vector3(Column[0].X, Column[1].X, Column[2].X),
				Vector3(Column[0].Y, Column[1].Y, Column[2].Y),
				Vector3(Column[0].Z, Column[1].Z, Column[2].Z));
		}


		FORCEINLINE constexpr Matrix3x3 operator*(const float scalar) const
		{
			return Matrix3x3(Column[0] * scalar, Column[1] * scalar, Column[2] * scalar);
		}

		FORCEINLINE const Vector3& operator[](const BYTE idx) const
		{
			return Column[idx];
		}
		FORCEINLINE constexpr Matrix3x3 operator*(const Matrix3x3& mat) const
		{
			Matrix3x3 transpose = GetTranspose();
			return Matrix3x3(
				Vector3(transpose[0].Dot(mat[0]), transpose[1].Dot(mat[0]), transpose[2].Dot(mat[0])),
				Vector3(transpose[0].Dot(mat[1]), transpose[1].Dot(mat[1]), transpose[2].Dot(mat[1])),
				Vector3(transpose[0].Dot(mat[2]), transpose[1].Dot(mat[2]), transpose[2].Dot(mat[2]))
				);
		}

		FORCEINLINE constexpr Vector3 operator*(const Vector3& vt3) const
		{
			Matrix3x3 transpose = GetTranspose();
			return Vector3(transpose[0].Dot(vt3), transpose[1].Dot(vt3), transpose[2].Dot(vt3));
		}

		FORCEINLINE constexpr float Det() const
		{
			return Column[0].X * (Column[1].Y * Column[2].Z - Column[1].Z * Column[2].Y) - Column[1].X * (Column[0].Y * Column[2].Z - Column[0].Z * Column[2].Y) + Column[2].X * (Column[0].Y * Column[1].Z - Column[0].Z * Column[1].Y);
		}

		FORCEINLINE constexpr bool GetReverse(Matrix3x3& outMat) const
		{
			float det = Det();
			if (det == 0)
			{
				return false;
			}
			else
			{
				float indet = 1 / det;
				outMat = Matrix3x3(
					Vector3(Column[1].Y * Column[2].Z - Column[2].Y * Column[1].Z, Column[2].Y * Column[0].Z - Column[0].Y * Column[2].Z, Column[0].Y * Column[1].Z - Column[1].Y * Column[0].Z),
					Vector3(Column[2].X * Column[1].Z - Column[1].X * Column[2].Z, Column[0].X * Column[2].Z - Column[2].X * Column[0].Z, Column[1].X * Column[0].Z - Column[0].X * Column[1].Z),
					Vector3(Column[1].X * Column[2].Y - Column[2].X * Column[1].Y, Column[2].X * Column[0].Y - Column[0].X * Column[2].Y, Column[0].X * Column[1].Y - Column[1].X * Column[0].Y))* indet;
				return true;
			}
		}

		const static Matrix3x3 Identity;
		const static int Rank = 3;
	};
}