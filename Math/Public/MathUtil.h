#pragma once

#include <math.h>
#include <array>
#include <string>
#include <vector>
#include <random>

#define FORCEINLINE __forceinline
#define SMALL_NUMBER 1.e-6f

typedef unsigned char		BYTE;
typedef unsigned int		UINT32;
typedef signed long long	INT64;


namespace HM
{
	struct Math
	{
		static constexpr float PI = 3.14159265358979323846f;
		static constexpr float TwoPI = PI * 2.f;

		static std::mt19937 Generator;
		FORCEINLINE static constexpr float GetDecimal(float value)
		{
			return value - (int)value;
		}

		FORCEINLINE static constexpr float Clamp(float value, float min, float max) {
			if (value < min) {
				return min;
			}
			else if (value > max) {
				return max;
			}
			else {
				return value;
			}
		}

		FORCEINLINE static constexpr float Deg2Rad(float deg) 
		{
			return deg * PI / 180.f;
		}


		FORCEINLINE static constexpr float Rad2Deg(float rad)
		{
			return rad * 180.f / PI;
		}


		FORCEINLINE static float Sin(float deg)
		{
			return sinf(Deg2Rad(deg));
		}
		FORCEINLINE static float Cos(float deg)
		{
			return cosf(Deg2Rad(deg));
		}
		FORCEINLINE static float Tan(float deg) {
			return tanf(Deg2Rad(deg));
		}

		static constexpr float DegRange(float deg)
		{
			if (deg < 0)
			{
				while (deg < 0)
				{
					deg += 360.f;
				}
			}

			if (deg >= 360)
			{
				while (deg >= 360)
				{
					deg -= 360.f;
				}
			}

			return deg;
		}

		static constexpr float DegDistance(float start, float desti)
		{
			start = DegRange(start);
			desti = DegRange(desti);

			if (start == desti)
			{
				return 0;
			}

			float sign = 1;

			float distance = abs(start - desti);
			if (distance > 180)
			{
				distance = 360 - distance;
				sign *= -1;
			}
			if (start > desti) {
				sign *= -1;
			}

			return distance * sign;
		}

		FORCEINLINE static constexpr float Linear(float start, float end, float per)
		{
			return start + (end - start) * per;
		}


		FORCEINLINE static constexpr float Min3(float f1, float f2, float f3)
		{
			float min = f1;
			if (min > f2) {
				min = f2;
			}
			if (min > f3) {
				min = f3;
			}
			return min;
		}

		FORCEINLINE static constexpr float Max3(float f1, float f2, float f3)
		{
			float max = f1;
			if (max < f2) {
				max = f2;
			}
			if (max < f3) {
				max = f3;
			}
			return max;
		}

		FORCEINLINE static float Randomf(float min, float max)
		{
			std::uniform_real_distribution<float> random(min, max);
			return random(Generator);
		}

		template<class T>
		FORCEINLINE static constexpr T Abs(const T value) 
		{
			return value < 0 ? -value : value;
		}

		FORCEINLINE static constexpr bool CheckRangeSM(const float f1, const float f2)
		{
			return Abs(f1 - f2) < SMALL_NUMBER;
		}

		FORCEINLINE static constexpr bool CheckRange(const float min, const float max, const float value)
		{
			return value >= min && value <= max;
		}

		FORCEINLINE static constexpr float Square(float value)
		{
			return value * value;
		}		
	};
}
