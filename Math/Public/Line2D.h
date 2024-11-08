#pragma once
namespace HM
{
	struct Line2D
	{
	public:
		Line2D(Vector2& pos, Vector2& pos2)
		{
			_Pos = pos;
			_Pos2 = pos2;
		}

		Vector2 _Pos;
		Vector2 _Pos2;
	};
}