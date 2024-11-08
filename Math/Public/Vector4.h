#pragma once

#include "MathUtil.h"

namespace HM
{
	struct Vector4
	{
		FORCEINLINE constexpr Vector4() = default;
		FORCEINLINE explicit constexpr Vector4(int x, int y, int z, int w) :X((float)x), Y((float)y), Z((float)z), W((float)w) {}
		FORCEINLINE explicit constexpr Vector4(float x, float y, float z, float w) : X(x), Y(y), Z(z), W(w) {}
		FORCEINLINE explicit constexpr Vector4(const Vector3& vt3, bool isPoint = true) : X(vt3.X), Y(vt3.Y), Z(vt3.Z) { W = isPoint ? 1.f : 0.f; }
		FORCEINLINE explicit constexpr Vector4(const Vector2& vt2, bool isPoint = true) : X(vt2.X), Y(vt2.Y), Z(0.f) { W = isPoint ? 1.f : 0.f; }

	public:
		static constexpr BYTE Dimension = 4;

		union
		{
			struct {
				float X, Y, Z, W;
			};

			std::array<float, Dimension> Scalars = { 0.f,0.f,0.f,0.f };
		};

		FORCEINLINE constexpr bool operator==(const Vector4& vt4) const
		{
			return X == vt4.X && Y == vt4.Y && Z == vt4.Z && W == vt4.W;
		}
		FORCEINLINE constexpr Vector4 operator*(float scalar) const
		{
			return Vector4(X * scalar, Y * scalar, Z * scalar, W * scalar);
		}
		FORCEINLINE constexpr void operator*=(float scalar)
		{
			X *= scalar;
			Y *= scalar;
			Z *= scalar;
			W *= scalar;
		}
		FORCEINLINE constexpr Vector4 operator+(const Vector4& vt4) const
		{
			return Vector4(X + vt4.X, Y + vt4.Y, Z + vt4.Z, W + vt4.W);
		}
		FORCEINLINE constexpr void operator+=(const Vector4& vt4)
		{
			X += vt4.X;
			Y += vt4.Y;
			Z += vt4.Z;
			W += vt4.W;
		}
		FORCEINLINE constexpr Vector4 operator-(const Vector4& vt4) const
		{
			return Vector4(X - vt4.X, Y - vt4.Y, Z - vt4.Z, W - vt4.W);
		}
		FORCEINLINE constexpr void operator-=(const Vector4& vt4)
		{
			X -= vt4.X;
			Y -= vt4.Y;
			Z -= vt4.Z;
			W -= vt4.W;
		}


		FORCEINLINE float Size() const
		{
			return sqrtf(X * X + Y * Y + Z * Z + W * W);
		}

		FORCEINLINE constexpr float Dot(const Vector4& vt4) const
		{
			return X * vt4.X + Y * vt4.Y + Z * vt4.Z + W * vt4.W;;
		}
		FORCEINLINE constexpr Vector3 GetVt3() const
		{
			return Vector3(X, Y, Z);
		}
		FORCEINLINE constexpr Vector2 GetVt2() const
		{
			return Vector2(X, Y);
		}

		FORCEINLINE constexpr void MulXYZ(float value) {
			X *= value;
			Y *= value;
			Z *= value;
		}
		FORCEINLINE constexpr void MulXY(float value) {
			X *= value;
			Y *= value;
		}
		FORCEINLINE static constexpr float GetT(const Vector4& v1, const Vector4& v2, const float value, const int axis)
		{
			switch (axis)
			{
			case 0:
				return (value - v2.X) / (v1.X - v2.X);
			case 1:
				return (value - v2.Y) / (v1.Y - v2.Y);
			case 2:
				return (value - v2.Z) / (v1.Z - v2.Z);
			case 3:
				return (value - v2.W) / (v1.W - v2.W);
			}
			return 0;
		}

		std::string GetStr() 
		{
			return std::to_string(X) + ", " + std::to_string(Y) + ", " + std::to_string(Z) + ", " + std::to_string(W);
		}
		const static Vector4 Zero;
		const static Vector4 One;
		const static Vector4 UnitX;
		const static Vector4 UnitY;
		const static Vector4 UnitZ;
		const static Vector4 UnitW;
	};

}

