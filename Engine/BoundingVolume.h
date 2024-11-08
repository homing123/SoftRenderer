#pragma once

#include "EngineUtil.h"
namespace HM
{
	enum BoundingVolumeKind
	{
		Sphere = 0,
		Box = 1,
	};

	struct BoundingVolume
	{
	public:
		BoundingVolumeKind _BVKind;
		virtual BoundCheckResult BoundCheck(const Frustum& frustum, int idx, int id)
		{
			return BoundCheckResult::Outside;
		}

		virtual void GetBoxInfo(Vector3& out_Min, Vector3& out_Max)
		{

		}
		virtual void GetCircleInfo(Vector3& out_Center, float& out_Radius)
		{

		}
	};
}