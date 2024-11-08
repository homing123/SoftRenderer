#pragma once
#include "MathUtil.h"

namespace HM
{
	struct Vertex2D
	{
	public:
		Vertex2D() = default;
		Vertex2D(Vector2& pos, LinearColor& color, Vector2& uv)
		{
			Pos = pos;
			Color = color;
			UV = uv;
		}

		Vector2 Pos;
		LinearColor Color;
		Vector2 UV;

	};
}