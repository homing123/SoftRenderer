#pragma once
#include "EngineUtil.h"

namespace HM
{
	class Transform2D
	{
	public:
		void SetPosition(const Vector2& pos) { _Position = pos; }
		void AddPosition(const Vector2& pos) { _Position += pos; }
		void SetScale(const Vector2& scale) { _Scale = scale; }
		void AddScale(const Vector2& scale) { _Scale += scale; }
		void SetRotation(const float& degree) { _Rotation = degree; Update();}
		void AddRotation(const float& degree) { _Rotation += degree; Update();}

		Vector2 GetPos() const { return _Position; }
		Vector2 GetScale() const { return _Scale; }
		float GetRotataion() const { return _Rotation; }

		FORCEINLINE Matrix3x3 GetModelingMatrix()const 
		{
			return Matrix3x3(_Scale.X * _Right.X, _Scale.Y * _Up.X, _Position.X,
							 _Scale.X * _Right.Y, _Scale.Y * _Up.Y, _Position.Y,
							 0,				      0,			    1);
		}
	private:
		Vector2 _Position = Vector2::Zero;
		Vector2 _Scale = Vector2::One;
		float _Rotation = 0.f;
		Vector2 _Right = Vector2(0, 1);
		Vector2 _Up = Vector2(1, 0);
		void Update() {
			_Rotation = Math::DegRange(_Rotation);

			float sin = Math::Sin(_Rotation);
			float cos = Math::Cos(_Rotation);

			_Right = Vector2(cos, sin);
			_Up = Vector2(-sin, cos);
		}
	};
}