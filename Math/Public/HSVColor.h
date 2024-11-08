#pragma once
#include "MathUtil.h"
namespace HM
{
	struct HSVColor
	{
	public :
		HSVColor() = default;
		HSVColor(float h, float s, float v)
		{
			H = h;
			S = s;
			V = v;
		}


		float H; // 0~1
		float S; // 0~1
		float V; // 0~1

		
		Color32 ToColor32() 
		{
			//h 0 = red 120 = green 240 = blue
			//s 0 = white 1
			float r = Math::Clamp(abs(Math::DegDistance(180.f, H * 360.f)) / 60 - 1, 0, 1);
			float g = Math::Clamp(abs(Math::DegDistance(300.f, H * 360.f)) / 60 - 1, 0, 1);
			float b = Math::Clamp(abs(Math::DegDistance(60.f, H * 360.f)) / 60 - 1, 0, 1);

			//현재 s = 1 일때 rgb값임
			//s가 0이될수록 rgb = 1,1,1이 됨

			r = Math::Linear(1, r, S);
			g = Math::Linear(1, g, S);
			b = Math::Linear(1, b, S);

			//v가 0이면 black v = 1이면 그대로

			r = Math::Linear(0, r, V);
			g = Math::Linear(0, g, V);
			b = Math::Linear(0, b, V);

			return LinearColor(r, g, b).ToColor32();
		}
		LinearColor ToLinearColor()
		{
			float r = Math::Clamp(abs(Math::DegDistance(180.f, H * 360.f)) / 60 - 1, 0, 1);
			float g = Math::Clamp(abs(Math::DegDistance(300.f, H * 360.f)) / 60 - 1, 0, 1);
			float b = Math::Clamp(abs(Math::DegDistance(60.f, H * 360.f)) / 60 - 1, 0, 1);
			r = Math::Linear(1, r, S);
			g = Math::Linear(1, g, S);
			b = Math::Linear(1, b, S);
			r = Math::Linear(0, r, V);
			g = Math::Linear(0, g, V);
			b = Math::Linear(0, b, V);

			return LinearColor(r, g, b);
		}

	};
}