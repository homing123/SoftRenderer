#pragma once

#include "MathUtil.h"
#include "Vector2.h"

namespace HM
{
	struct Matrix2x2
	{
	public:
		Vector2 Column[2] = { Vector2::UnitX, Vector2::UnitY };

		FORCEINLINE constexpr Matrix2x2() = default;
		FORCEINLINE constexpr Matrix2x2(float _00, float _01, float _10, float _11)
		{
			Column[0].X = _00;
			Column[0].Y = _10;
			Column[1].X = _01;
			Column[1].Y = _11;
		}
		FORCEINLINE constexpr  Matrix2x2(const Vector2& v1, const Vector2& v2)
		{
			Column[0] = v1;
			Column[1] = v2;
		}

		FORCEINLINE constexpr Matrix2x2 GetTranspose() const
		{
			return Matrix2x2(Vector2(Column[0].X, Column[1].X),
				Vector2(Column[0].Y, Column[1].Y));
		}


		FORCEINLINE constexpr Matrix2x2 operator*(const float scalar) const
		{
			return Matrix2x2(Column[0] * scalar, Column[1] * scalar);
		}

		FORCEINLINE const Vector2& operator[](const BYTE idx) const
		{
			return Column[idx];
		}
		FORCEINLINE constexpr Matrix2x2 operator*(const Matrix2x2& mat) const
		{
			Matrix2x2 transpose = GetTranspose();
			return Matrix2x2(
				Vector2(transpose[0].Dot(mat[0]), transpose[1].Dot(mat[0])),
				Vector2(transpose[0].Dot(mat[1]), transpose[1].Dot(mat[1]))
				);
		}

		FORCEINLINE constexpr Vector2 operator*(const Vector2& vt2) const
		{
			Matrix2x2 transpose = GetTranspose();
			return Vector2(transpose[0].Dot(vt2), transpose[1].Dot(vt2));
		}

		FORCEINLINE constexpr float Det() const
		{
			return Column[0].X * Column[1].Y - Column[1].X * Column[0].Y;
		}

		FORCEINLINE constexpr bool GetReverse(Matrix2x2& outMat) const
		{
			float det = Det();
			if (det == 0)
			{
				return false;
			}
			else
			{
				float indet = 1 / det;
				outMat = Matrix2x2(
					Vector2(Column[1].Y, -Column[0].Y),
					Vector2(-Column[1].X, Column[0].X)) * indet;
				return true;
			}
		}

		const static Matrix2x2 Identity;
		const static int Rank = 2;
	};

}