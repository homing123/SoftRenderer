#pragma once
#include "EngineUtil.h"

namespace HM
{
	class Transform
	{
	public:
		void SetPosition(const Vector3& pos) { _Position = pos; }
		void AddPosition(const Vector3& pos) { _Position += pos; }
		void SetScale(const Vector3& scale) { _Scale = scale; }
		void AddScale(const Vector3& scale) { _Scale += scale; }
		void SetRotation(const Vector3& euler) 
		{ 
			_Rotation = euler;
			Update(); 
		}
		void AddRotationX(const float degree)
		{ 
			_Rotation.X += degree;
			Update();
		}
		void AddRotationY(const float degree)
		{
			_Rotation.Y += degree;
			Update();
		}
		void AddRotationZ(const float degree)
		{
			_Rotation.Z += degree;
			Update();
		}
		void AddRotation(const Vector3& axis, const float degree) 
		{
			Vector3 axisn = axis.GetNormal();
			Vector3 axisPos = Vector3::Zero;
			Vector3 curPos = _Position;
			Vector3 circleCenter = axisn * axisn.Dot(curPos - axisPos);

			//curPos 와 circleCenter 를 circleCenter 만큼 빼준 후 계산 하고 결과에 circleCenter를 더함
			//회전 중심을 0,0,0으로 땡겨서 계산 후 다시원위치로 옮겨줌

			curPos -= circleCenter;
			Vector3 right = curPos;
			Vector3 front = right.CrossProduct(axisn);
			
			float sin = Math::Sin(degree);
			float cos = Math::Cos(degree);

			Vector3 rotPos = (right * cos - front * sin ) + circleCenter;
			_Position = rotPos;
		}
		void AddRotation(const Vector3& axis, const Vector3& axisPos, const float degree)
		{
			Vector3 axisn = axis.GetNormal();
			Vector3 curPos = _Position;
			Vector3 circleCenter = axisn * axisn.Dot(curPos - axisPos);

			curPos -= circleCenter;
			Vector3 right = curPos;
			Vector3 front = right.CrossProduct(axisn);

			float sin = Math::Sin(degree);
			float cos = Math::Cos(degree);

			Vector3 rotPos = (right * cos - front * sin) + circleCenter;
			_Position = rotPos;
		}
		void LookAt(const Vector3& targetPos, const Vector3& Up = Vector3(0,1,0))
		{
			Vector3 l_x, l_y, l_z;
			l_z = (targetPos - _Position).GetNormal();

			if (l_z.Y >= (1 - (SMALL_NUMBER)))
			{
				l_x = Vector3(1, 0, 0);
			}
			else {
				l_x = Up.CrossProduct(l_z).GetNormal();
			}
			l_y = l_z.CrossProduct(l_x);

			_Right = l_x;
			_Up = l_y;
			_Front = l_z;
		}

		Vector3 GetPos() const { return _Position; }
		Vector3 GetScale() const { return _Scale; }
		Vector3 GetEuler() const { return _Rotation; }
		Vector3 GetFront() const { return _Front; }
		Vector3 GetUp() const { return _Up; }
		Vector3 GetRight() const { return _Right; }

		FORCEINLINE Matrix4x4 GetModelingMatrix()const
		{
			return Matrix4x4(
				Vector4(_Right * _Scale.X, false),
				Vector4(_Up * _Scale.Y, false),
				Vector4(_Front * _Scale.Z, false),
				Vector4(_Position, true));
		}
	private:
		Vector3 _Position = Vector3::Zero;
		Vector3 _Scale = Vector3::One;
		Vector3 _Rotation = Vector3::Zero;
		Vector3 _Right = Vector3(1, 0, 0);
		Vector3 _Up = Vector3(0, 1, 0);
		Vector3 _Front = Vector3(0, 0, 1);
		void Update() {
			_Rotation.X = Math::DegRange(_Rotation.X);
			_Rotation.Y = Math::DegRange(_Rotation.Y);
			_Rotation.Z = Math::DegRange(_Rotation.Z);

			float sx = Math::Sin(_Rotation.X);
			float cx = Math::Cos(_Rotation.X);
			float sy = Math::Sin(_Rotation.Y);
			float cy = Math::Cos(_Rotation.Y);
			float sz = Math::Sin(_Rotation.Z);
			float cz = Math::Cos(_Rotation.Z);

			//My * Mx * Mz 순서임 유니티와 동일
			//돌아간 상태에서의 회전이라고 생각 즉 My는 항등행렬 에서의 회전이므로 영향안받음
			//순서상 Mx 는 My가 돌아간 후의 회전임 즉 My는 무조건 월드와 동일하게 돌고 Mx는 My회전에만 영향받고 Mz는 My, Mx에 모두 영향받음
			_Right = Vector3(cy * cz + sy * sx * sz, cx * sz, -sy * cz + cy * sx * sz);
			_Up = Vector3(cy *  (-sz) + sy * sx * cz, cx * cz, (-sy)*(-sz) + cy * sx * cz);
			_Front = Vector3(sy * cx, -sx, cy * cx);
		}
	};
}