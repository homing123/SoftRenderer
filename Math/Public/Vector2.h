#pragma once

#include "MathUtil.h"

namespace HM 
{
	struct Vector2
	{
		FORCEINLINE constexpr Vector2() = default;
		FORCEINLINE explicit constexpr Vector2(int x, int y) :X((float)x), Y((float)y) {}
		FORCEINLINE explicit constexpr Vector2(float x, float y) : X(x), Y(y) {}

	public:
		static constexpr BYTE Dimension = 2;

		union
		{
			struct {
				float X, Y;
			};
			
			std::array<float, Dimension> Scalars = { 0.f,0.f };
		};

		FORCEINLINE constexpr bool operator ==(const Vector2& vt2) const
		{
			return X == vt2.X && Y == vt2.Y;
		}
		FORCEINLINE constexpr Vector2 operator*(float scalar) const
		{
			return Vector2(X * scalar, Y * scalar);
		}
		FORCEINLINE constexpr void operator*=(float scalar)
		{
			X *= scalar;
			Y *= scalar;
		}
		FORCEINLINE constexpr Vector2 operator-(const Vector2& vt2) const
		{
			return Vector2(X - vt2.X, Y - vt2.Y);
		}
		FORCEINLINE constexpr void operator-=(const Vector2& vt2)
		{
			X -= vt2.X;
			Y -= vt2.Y;
		}
		FORCEINLINE constexpr Vector2 operator+(const Vector2& vt2) const
		{
			return Vector2(X + vt2.X, Y + vt2.Y);
		}
		FORCEINLINE constexpr void operator+=(const Vector2& vt2)
		{
			X += vt2.X;
			Y += vt2.Y;
		}
		
		FORCEINLINE float Angle() const
		{
			return Math::Rad2Deg(atan2f(Y, X));
		}
		void Rotation(const Vector2& axis, const float& angle)
		{
			float sin = Math::Sin(angle);
			float cos = Math::Cos(angle);

			float _x = X * cos - Y * sin;
			float _y = X * sin + Y * cos;
			X = _x;
			Y = _y;
		}

		FORCEINLINE float Size() const
		{
			return sqrtf(X * X + Y * Y);
		}


		FORCEINLINE constexpr Vector2 ToPolarCoordinate() const
		{
			return Vector2(Size(), Angle());
		}
		FORCEINLINE constexpr Vector2 ToCartesianCoordinate() const
		{
			return Vector2(Math::Cos(Y) * X, Math::Sin(Y) * X);
		}

		FORCEINLINE constexpr float Dot(const Vector2& vt2) const
		{
			return X * vt2.X + Y * vt2.Y;
		}

		FORCEINLINE Vector2 GetNormal() const
		{
			float size = Size();
			return Vector2(X / size, Y / size);
		}
		FORCEINLINE void Normalize()
		{
			float size = Size();
			X = X / size;
			Y = Y / size;
		}

		FORCEINLINE static constexpr float GetT(const Vector2& v1, const Vector2& v2, const float value, const int axis)
		{
			switch (axis)
			{
			case 0:
				return (value - v2.X) / (v1.X - v2.X);
			case 1:
				return (value - v2.Y) / (v1.Y - v2.Y);
			}
			return 0;
		}
		FORCEINLINE static constexpr float GetY(const Vector2& v1, const Vector2& v2, const float x)
		{
			//small * t + large * (1 - t) = x
			float t = 0;
			if (v1.X < v2.X)
			{
				t = (x - v2.X) / (v1.X - v2.X);
				return v1.Y * t + v2.Y * (1 - t);

			}
			else if(v1.X > v2.X)
			{
				t = (x - v1.X) / (v2.X - v1.X);
				return v2.Y * t + v1.Y * (1 - t);
			}
			else
			{
				return 0;
			}
		}
		FORCEINLINE static constexpr float GetX(const Vector2& v1, const Vector2& v2, const float y)
		{
			//small * t + large * (1 - t) = y
			float t = 0;
			if (v1.Y < v2.Y)
			{
				t = (y - v2.Y) / (v1.Y - v2.Y);
				return v1.X * t + v2.X * (1 - t);

			}
			else if (v1.Y > v2.Y)
			{
				t = (y - v1.Y) / (v2.Y - v1.Y);
				return v2.X * t + v1.X * (1 - t);
			}
			else
			{
				return 0;
			}
		}

		const static Vector2 Zero;
		const static Vector2 One;
		const static Vector2 UnitX;
		const static Vector2 UnitY;
	};
}

