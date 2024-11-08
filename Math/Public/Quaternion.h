#pragma once

#include "MathUtil.h"
namespace HM
{
	struct Quaternion
	{
	public:
		//FORCEINLINE constexpr float RealPart() const { return W; }
		//FORCEINLINE constexpr Vector3 ImaginaryPart() const { return Vector3(X, Y, Z); }

		//float X = 0.f;
		//float Y = 0.f;
		//float Z = 0.f;
		//float W = 1.f;

		//FORCEINLINE constexpr Vector3 RotateVector(const Vector3& inVector) const
		//{
		//	Vector3 q(X, Y, Z);
		//	Vector3 t = q.CrossProduct(inVector) * 2.f;
		//	Vector3 result = inVector + t * W + q.CrossProduct(t);
		//	return result;
		//}

		//FORCEINLINE constexpr Quaternion operator*(const Quaternion& inQuat) const
		//{
		//	Quaternion result;
		//	Vector3 v1(X, Y, Z), v2(inQuat.X, inQuat.Y, inQuat.Z);
		//	result.W = W * inQuat.W - v1.Dot(v2);
		//	Vector3 resultV = v2 * W + v1 * inQuat.W + v1.CrossProduct(v2);
		//	result.X = resultV.X;
		//	result.Y = resultV.Y;
		//	result.Z = resultV.Z;
		//	return result;
		//}
		//FORCEINLINE constexpr Vector3 operator*(const Vector3& inVector) const
		//{
		//	return RotateVector(inVector);
		//}
		//FORCEINLINE constexpr void FromRotation(const Vector3& inRotation) 
		//{
		//	float sx = Math::Sin(inRotation.X * 0.5f);
		//	float sy = Math::Sin(inRotation.Y * 0.5f);
		//	float sz = Math::Sin(inRotation.Z * 0.5f);
		//	float cx = Math::Cos(inRotation.X * 0.5f);
		//	float cy = Math::Cos(inRotation.Y * 0.5f);
		//	float cz = Math::Cos(inRotation.Z * 0.5f);

		//	W = cx * cy * cz + sx * sy * sz;
		//	X = sx * cy * cz + cx * sy * sz;
		//	Y = sy * cx * cz - cy * sx * sz;
		//	Z = sz * cx * cy - cz * sx * sy;
		//}
	};


}