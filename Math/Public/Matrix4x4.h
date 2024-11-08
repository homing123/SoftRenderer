#pragma once

#include "MathUtil.h"

namespace HM
{
	struct Matrix4x4
	{
	public:
		float _00 = 0, _01 = 0, _02 = 0, _03 = 0, _10 = 0, _11 = 0, _12 = 0, _13 = 0, _20 = 0, _21 = 0, _22 = 0, _23 = 0, _30 = 0, _31 = 0, _32 = 0, _33 = 0;

		FORCEINLINE constexpr Matrix4x4() = default;
		FORCEINLINE constexpr Matrix4x4(float f00, float f01, float f02, float f03, float f10, float f11, float f12, float f13, float f20, float f21, float f22, float f23, float f30, float f31, float f32, float f33)
		{
			_00 = f00;
			_01 = f01;
			_02 = f02;
			_03 = f03;
			_10 = f10;
			_11 = f11;
			_12 = f12;
			_13 = f13;
			_20 = f20;
			_21 = f21;
			_22 = f22;
			_23 = f23;
			_30 = f30;
			_31 = f31;
			_32 = f32;
			_33 = f33;
		}
		FORCEINLINE constexpr Matrix4x4(const Vector4& v1, const Vector4& v2, const Vector4& v3, const Vector4& v4)
		{
			_00 = v1.X;
			_01 = v2.X;
			_02 = v3.X;
			_03 = v4.X;
			_10 = v1.Y;
			_11 = v2.Y;
			_12 = v3.Y;
			_13 = v4.Y;
			_20 = v1.Z;
			_21 = v2.Z;
			_22 = v3.Z;
			_23 = v4.Z;
			_30 = v1.W;
			_31 = v2.W;
			_32 = v3.W;
			_33 = v4.W;
		}

		FORCEINLINE constexpr Matrix4x4 GetTranspose() const
		{
			return Matrix4x4
			(
				_00, _10, _20, _30,
				_01, _11, _21, _31,
				_02, _12, _22, _32,
				_03, _13, _23, _33
			);
		}


		FORCEINLINE constexpr Matrix4x4 operator*(const float scalar) const
		{
			return Matrix4x4(_00 * scalar, _01 * scalar, _02 * scalar, _03 * scalar, 
							 _10 * scalar, _11 * scalar, _12 * scalar, _13 * scalar, 
							 _20 * scalar, _21 * scalar, _22 * scalar, _23 * scalar, 
							 _30 * scalar, _31 * scalar, _32 * scalar, _33 * scalar);
		}

		FORCEINLINE const float operator[](const BYTE idx) const
		{
			switch (idx)
			{
			case 0:
				return _00;
			case 1:
				return _01;
			case 2:
				return _02;
			case 3:
				return _03;
			case 4:
				return _10;
			case 5:
				return _11;
			case 6:
				return _12;
			case 7:
				return _13;
			case 8:
				return _20;
			case 9:
				return _21;
			case 10:
				return _22;
			case 11:
				return _23;
			case 12:
				return _30;
			case 13:
				return _31;
			case 14:
				return _32;
			case 15:
				return _33;
			}
			return 0;
		}


		FORCEINLINE const Vector4 GetColumn(const int idx) const
		{
			switch (idx)
			{
			case 0:
				return Vector4(_00, _10, _20, _30);
			case 1:
				return Vector4(_01, _11, _21, _31);
			case 2:
				return Vector4(_02, _12, _22, _32);
			case 3:
				return Vector4(_03, _13, _23, _33);
			}
			return Vector4::Zero;
		}
		FORCEINLINE constexpr Matrix4x4 operator*(const Matrix4x4& mat) const
		{
			return Matrix4x4
			(
				_00 * mat._00 + _01 * mat._10 + _02 * mat._20 + _03 * mat._30,
				_00 * mat._01 + _01 * mat._11 + _02 * mat._21 + _03 * mat._31,
				_00 * mat._02 + _01 * mat._12 + _02 * mat._22 + _03 * mat._32,
				_00 * mat._03 + _01 * mat._13 + _02 * mat._23 + _03 * mat._33,

				_10* mat._00 + _11 * mat._10 + _12 * mat._20 + _13 * mat._30,
				_10* mat._01 + _11 * mat._11 + _12 * mat._21 + _13 * mat._31,
				_10* mat._02 + _11 * mat._12 + _12 * mat._22 + _13 * mat._32,
				_10* mat._03 + _11 * mat._13 + _12 * mat._23 + _13 * mat._33,

				_20* mat._00 + _21 * mat._10 + _22 * mat._20 + _23 * mat._30,
				_20* mat._01 + _21 * mat._11 + _22 * mat._21 + _23 * mat._31,
				_20* mat._02 + _21 * mat._12 + _22 * mat._22 + _23 * mat._32,
				_20* mat._03 + _21 * mat._13 + _22 * mat._23 + _23 * mat._33,

				_30* mat._00 + _31 * mat._10 + _32 * mat._20 + _33 * mat._30,
				_30* mat._01 + _31 * mat._11 + _32 * mat._21 + _33 * mat._31,
				_30* mat._02 + _31 * mat._12 + _32 * mat._22 + _33 * mat._32,
				_30* mat._03 + _31 * mat._13 + _32 * mat._23 + _33 * mat._33
			);
		}

		FORCEINLINE constexpr Vector4 operator*(const Vector4& vt4) const
		{
			return Vector4(_00 * vt4.X + _01 * vt4.Y + _02 * vt4.Z + _03 * vt4.W,
						   _10 * vt4.X + _11 * vt4.Y + _12 * vt4.Z + _13 * vt4.W, 
						   _20 * vt4.X + _21 * vt4.Y + _22 * vt4.Z + _23 * vt4.W, 
						   _30 * vt4.X + _31 * vt4.Y + _32 * vt4.Z + _33 * vt4.W);
		}
		FORCEINLINE constexpr Vector3 operator*(const Vector3& vt3) const
		{
			return Vector3(_00 * vt3.X + _01 * vt3.Y + _02 * vt3.Z,
						   _10 * vt3.X + _11 * vt3.Y + _12 * vt3.Z,
						   _20 * vt3.X + _21 * vt3.Y + _22 * vt3.Z);
		}

		const static Matrix4x4 Identity;
		const static int Rank = 3;

	};
}