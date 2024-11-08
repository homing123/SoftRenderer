#pragma once
namespace HM
{
	struct Ray2D
	{
	public:
		Ray2D(Vector2 startPos, Vector2 direction)
		{
			_StartPos = startPos;
			_Direction = direction;
		}

		Vector2 _StartPos;
		Vector2 _Direction;
	};
}