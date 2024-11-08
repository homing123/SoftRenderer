#pragma once

#include "MathUtil.h"
#include "Color32.h"
namespace HM
{
	struct  LinearColor
	{
	public :
		LinearColor() = default;
		FORCEINLINE explicit constexpr LinearColor(float InR, float InG, float InB, float InA = 1.f) : R(InR), G(InG), B(InB), A(InA) {}


		LinearColor(int r, int g, int b, int a = 255)
		{
			R = (float)r / 255;
			G = (float)g / 255;
			B = (float)b / 255;
			A = (float)a / 255;
		}

		float R = 0.f;
		float G = 0.f;
		float B = 0.f;
		float A = 1.f;

		void ColorChange(float r, float g, float b, float a = 1) {
			R = r;
			G = g;
			B = b;
			A = a;
		}
		void ColorChange(int r, int g, int b, int a = 255) 
		{
			R = (float)r / 255;
			G = (float)g / 255;
			B = (float)b / 255;
			A = (float)a / 255;
		}

		float ToGrayScale() 
		{
			return 0.299f * R + 0.587f * G + 0.114f * B;
		}

		FORCEINLINE constexpr LinearColor operator+(LinearColor color) const
		{
			return LinearColor(R + color.R, G + color.G, B + color.B, A + color.A);
		}
		FORCEINLINE constexpr LinearColor operator* (float value) const
		{
			return LinearColor(R * value, G * value, B * value, A * value);
		}
		FORCEINLINE constexpr bool operator == (LinearColor color) const
		{
			return R == color.R && G == color.G && B == color.B && A == color.A;
		}
		FORCEINLINE constexpr bool operator != (LinearColor color) const
		{
			return !(R == color.R && G == color.G && B == color.B && A == color.A);
		}
		FORCEINLINE constexpr LinearColor GetMix(const LinearColor& mixcolor, const float value)
		{
			return LinearColor(R * (1 - value) +  mixcolor.R * value, G * (1 - value) + mixcolor.G * value, B * (1 - value) + mixcolor.B * value, A * (1 - value) + mixcolor.A * value);
		}
		Color32 ToColor32() const
		{
			BYTE r = (BYTE)(Math::Clamp(R, 0, 1) * 255);
			BYTE g = (BYTE)(Math::Clamp(G, 0, 1) * 255);
			BYTE b = (BYTE)(Math::Clamp(B, 0, 1) * 255);
			BYTE a = (BYTE)(Math::Clamp(A, 0, 1) * 255);

			return Color32(r, g, b, a);
		}
		Color32 ToColor32()
		{
			BYTE r = (BYTE)(Math::Clamp(R, 0, 1) * 255);
			BYTE g = (BYTE)(Math::Clamp(G, 0, 1) * 255);
			BYTE b = (BYTE)(Math::Clamp(B, 0, 1) * 255);
			BYTE a = (BYTE)(Math::Clamp(A, 0, 1) * 255);

			return Color32(r, g, b, a);
		}
		static const LinearColor Error;
		static const LinearColor White;
		static const LinearColor Black;
		static const LinearColor Gray;
		static const LinearColor Red;
		static const LinearColor Green;
		static const LinearColor Blue;
		static const LinearColor Yellow;
		static const LinearColor Cyan;
		static const LinearColor Magenta;
		static const LinearColor Empty;
	};


}
