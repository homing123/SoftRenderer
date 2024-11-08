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
		//ī�޶� ȸ�� ��� x
		return Matrix3x3(1, 0, -_Transform.GetPos().X, 0, 1, -_Transform.GetPos().Y, 0, 0, 1);
	}
	FORCEINLINE Matrix4x4 Camera::GetWorldToViewMat() const
	{
		//������,ȸ��,�̵�
		//TRS �ε� S�� �׵��
		//(TR)-1 = R-1 T-1
		//R���Լ��� �� ���������� ������ 0, ���̰� 1������ �̿��ؼ� ��ġ����� ������ΰ��� ��������
		//T�� �̵��� ������� �ݴ�������� �̵��̶� �̵���ǥ�� -�����ָ� ��

		//�Ʒ� ī�޶� �ٶ󺸴¹����� ������ ������� �������� 0���϶� ����
		//å = ��������ǥ�� ��������ǥ�迡�� ī�޶� -Z������ ���� X,Y���� ������ ����
		//���ڵ� = �޼���ǥ�� ī�޶� +Z������ ���� X,Y���� ������ ����
		//å���� Y���� 180�� �������� �� �׷��ʿ����

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

	//��ȯ �� x,y,z �� w������ ���������
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

	//��ȯ �� x,y,z �� w������ ���������
	FORCEINLINE Matrix4x4 Camera::GetWorldToNDCMat() const
	{
		//view = ī�޶� ���� ����
		//clip = view���� ����ü�� �ڸ� ����
		//ndc = clip������ xy = -1 ~ 1, z = 0 ~ 1 (å������ z = -1 ~ 1) ������ü�� ��ȯ�� ����

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
		//k = 1, l = 0 �� �Ǿ ����� ���°Ű�����
		//k �� l �� �׵���� ���̾ ��������� z������ �����ϴµ� �츰 ��������� z���� depth���� ���� ����ؼ� ����
		//k = 1; l = 0;
		
		return Matrix4x4(
			Vector4(right.X * ad, up.X * d, front.X * k, front.X),
			Vector4(right.Y * ad, up.Y * d, front.Y * k, front.Y),
			Vector4(right.Z * ad, up.Z * d, front.Z * k, front.Z),
			Vector4(-right.Dot(pos) * ad, -up.Dot(pos) * d, -front.Dot(pos) * k + l, -front.Dot(pos)));
	}
}