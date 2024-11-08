#pragma once

#include "EngineUtil.h"
namespace HM
{
	class Camera {
	public:
		Camera() = default;
		~Camera() {}

		Transform& GetTransform() { return _Transform; }
		const Transform& GetTransform() const { return _Transform; }

		const ScreenPoint& GetViewPortSize() const { return _ViewPortSize; }

		void SetViewPortSize(const ScreenPoint& viewportSize) {
			_ViewPortSize = viewportSize;
		}

		FORCEINLINE Matrix3x3 GetViewMat2D() const;
		FORCEINLINE Matrix4x4 GetWorldToViewMat() const;
		FORCEINLINE Matrix4x4 GetWorldToNDCMat() const;
		FORCEINLINE Matrix4x4 GetViewToNDCMat() const;

		void SetNearDis(const float neardis) 
		{
			_NearDis = neardis;
		}
		void SetFarDis(const float fardis) {
			_FarDis = fardis;
		}
		void SetFOV(const float fov) {
			_FOV = fov;
		}

		FORCEINLINE constexpr void GetNearFar(float& outNear, float& outFar) {
			outNear = _NearDis;
			outFar = _FarDis;
		}
		FORCEINLINE constexpr float GetFOV() {
			return _FOV;
		}

	private:
		Transform _Transform;
		ScreenPoint _ViewPortSize;
		float _NearDis = 0.05f;
		float _FarDis = 1000.f;
		float _FOV = 90.f ;
	};

	FORCEINLINE Matrix3x3 Camera::GetViewMat2D() const
	{
		//카메라 회전 고려 x
		return Matrix3x3(1, 0, -_Transform.GetPos().X, 0, 1, -_Transform.GetPos().Y, 0, 0, 1);
	}
	FORCEINLINE Matrix4x4 Camera::GetWorldToViewMat() const
	{
		//스케일,회전,이동
		//TRS 인데 S는 항듣원
		//(TR)-1 = R-1 T-1
		//R역함수는 각 기저벡터의 내적이 0, 길이가 1인점을 이용해서 전치행렬이 역행렬인것을 검증가능
		//T는 이동의 역행렬은 반대방향으로 이동이라 이동좌표에 -곱해주면 됨

		//아래 카메라가 바라보는방향의 기준은 모든축의 각도값이 0도일때 기준
		//책 = 오른손좌표계 오른손좌표계에선 카메라가 -Z방향을 봐야 X,Y축의 방향이 맞음
		//내코드 = 왼손좌표계 카메라가 +Z방향을 봐야 X,Y축의 방향이 맞음
		//책에선 Y축을 180도 돌리지만 난 그럴필요없음

		//world to view

		Vector3 pos = _Transform.GetPos();
		Vector3 front = _Transform.GetFront();
		Vector3 right = _Transform.GetRight();
		Vector3 up = _Transform.GetUp();

		return Matrix4x4(
			Vector4(right.X, up.X, front.X, 0.f),
			Vector4(right.Y, up.Y, front.Y, 0.f),
			Vector4(right.Z, up.Z, front.Z, 0.f),
			Vector4(-right.Dot(pos), -up.Dot(pos), -front.Dot(pos), 1.f));
	}

	//변환 후 x,y,z 에 w값으로 나눠줘야함
	FORCEINLINE Matrix4x4 Camera::GetViewToNDCMat() const
	{	

		float InvFN = 1 / (_FarDis - _NearDis);
		float viewportRatio = (float)_ViewPortSize.Y / _ViewPortSize.X;
		float Inv_fov = 1 / Math::Tan(_FOV * 0.5f);
		float k = (_FarDis + _NearDis) * InvFN;
		float l = (-2 * _FarDis * _NearDis) * InvFN;

		return Matrix4x4(
			Vector4(viewportRatio * Inv_fov, 0.f, 0.f, 0.f),
			Vector4(0.f, Inv_fov, 0.f, 0.f),
			Vector4(0.f,0.f,k,1.f),
			Vector4(0.f,0.f,l, 0.f));
	}

	//변환 후 x,y,z 에 w값으로 나눠줘야함
	FORCEINLINE Matrix4x4 Camera::GetWorldToNDCMat() const
	{
		//view = 카메라 기준 공간
		//clip = view에서 절두체로 자른 공간
		//ndc = clip공간을 xy = -1 ~ 1, z = 0 ~ 1 (책에서는 z = -1 ~ 1) 정육면체로 변환한 공간

		//world to ndc
		Vector3 pos = _Transform.GetPos();
		Vector3 front = _Transform.GetFront();
		Vector3 right = _Transform.GetRight();
		Vector3 up = _Transform.GetUp();

		float InvFN = 1 / (_FarDis - _NearDis);
		float a = (float)_ViewPortSize.Y / _ViewPortSize.X;
		float d = 1 / Math::Tan(_FOV * 0.5f);
		float k = (_FarDis + _NearDis) * InvFN;
		float l = (-2 * _FarDis * _NearDis) * InvFN;
		float ad = a * d;
		//k = 1, l = 0 이 되어도 상관이 없는거같은데
		//k 와 l 이 항등행렬 값이어도 결과벡터의 z값에만 관여하는데 우린 결과벡터의 z값에 depth값을 새로 계산해서 넣음
		//k = 1; l = 0;
		
		return Matrix4x4(
			Vector4(right.X * ad, up.X * d, front.X * k, front.X),
			Vector4(right.Y * ad, up.Y * d, front.Y * k, front.Y),
			Vector4(right.Z * ad, up.Z * d, front.Z * k, front.Z),
			Vector4(-right.Dot(pos) * ad, -up.Dot(pos) * d, -front.Dot(pos) * k + l, -front.Dot(pos)));
	}
}