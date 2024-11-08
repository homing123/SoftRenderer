#pragma once

#include "MathUtil.h"
namespace HM
{

	//ABGR
	struct Color32
	{
	public:
		FORCEINLINE constexpr Color32() :R(0), G(0), B(0), A(255) {}
		FORCEINLINE explicit constexpr Color32(BYTE r, BYTE g, BYTE b, BYTE a = 255) : R(r), G(g), B(b), A(a) {}
		FORCEINLINE explicit constexpr Color32(UINT32 color) : ColorValue(color) {}

		union
		{
			struct
			{
				BYTE B, G, R, A;
			};

			UINT32 ColorValue;
		};

		FORCEINLINE constexpr bool operator == (Color32 color) const
		{
			return R == color.R && G == color.G && B == color.B && A == color.A;
		}
		FORCEINLINE constexpr bool operator != (Color32 color) const
		{
			return !(R == color.R && G == color.G && B == color.B && A == color.A);
		}
		static const Color32 Error;
		static const Color32 White;
		static const Color32 Black;
		static const Color32 Gray;
		static const Color32 Red;
		static const Color32 Green;
		static const Color32 Blue;
		static const Color32 Yellow;
		static const Color32 Cyan;
		static const Color32 Magenta;

		//LinearColor ToLinearColorr()
		//{
		//	return LinearColor(R / 255.f, G / 255.f, B / 255.f, A / 255.f);
		//}

	};
}