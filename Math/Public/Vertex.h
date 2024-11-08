#pragma once
#include "MathUtil.h"
namespace HM
{
	struct Vertex {
	public:
		FORCEINLINE constexpr Vertex() = default;
		Vertex(const Vector4& pos, const Vector2& uv)
		{
			Pos = pos;
			UV = uv;
		}
		Vertex(const Vector4& pos)
		{
			Pos = pos;
			UV = Vector2::Zero;
		}
		Vector4 Pos;
		Vector2 UV;

	};
}