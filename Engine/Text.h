#pragma once

#include "EngineUtil.h"

namespace HM
{
	struct Text
	{
	public:
		Text() = default;
		Text(string str, Vector2 pos, Color32 color) 
		{
			_String = std::wstring(str.begin(), str.end());
			_Pos = pos;
			_Color = color;
		}
		Text(wstring str, Vector2 pos, Color32 color)
		{
			_String = str;
			_Pos = pos;
			_Color = color;
		}

		void ChangeString(string str)
		{
			_String = std::wstring(str.begin(), str.end());
		}
		wstring _String;
		Vector2 _Pos;
		Color32 _Color;
	};
}