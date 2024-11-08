#pragma once
namespace HM
{
	struct Segment2D
	{
	public:
		Segment2D() = default;
		Segment2D(Vector2 pos, Vector2 pos2)
		{
			_Pos = pos;
			_Pos2 = pos2;
		}

		Vector2 _Pos;
		Vector2 _Pos2;

		bool GetVt2_X(float x, Vector2& outVt2) 
		{
			if (_Pos.X == _Pos2.X && _Pos.X != x) 
			{
				return false;
			}
			else
			{
				float a = (x - _Pos2.X) / (_Pos.X - _Pos2.X);
				if (a < 0 || a > 1) 
				{
					return false;
				}
				outVt2.X = x;
				outVt2.Y = a * _Pos.Y + (1 - a) * _Pos2.Y;
				return true;
			}
		}

		bool GetVt2_Y(float y, Vector2& outVt2)
		{
			if (_Pos.Y == _Pos2.Y && _Pos.Y != y)
			{
				return false;
			}
			else
			{
				float a = (y - _Pos2.Y) / (_Pos.Y - _Pos2.Y);
				if (a < 0 || a > 1)
				{
					return false;
				}
				outVt2.Y = y;
				outVt2.X = a * _Pos.X + (1 - a) * _Pos2.X;
				return true;
			}
		}
	};
}