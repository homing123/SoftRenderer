#pragma once

#include "EngineUtil.h"
namespace HM
{
	struct BVSphere : BoundingVolume
	{
		BVSphere(Vector3* vertices)
		{
			_BVKind = BoundingVolumeKind::Sphere;

			int size = _msize(vertices) / sizeof(Vector3);
			Vector3 total;
			for (int i = 0; i < size; i++)
			{
				total += vertices[i];
			}

			_Center *= 1 / size;

			float dispow = 0;
			for (int i = 0; i < size; i++)
			{
				if (Vector3::DistancePow(_Center, vertices[i]) > dispow)
				{
					dispow = Vector3::DistancePow(_Center, vertices[i]);
				}
			}

			_Radius = sqrt(dispow);
		}

		Vector3 _Center = Vector3::Zero;
		float _Radius = 0;

		BoundCheckResult BoundCheck(const Frustum& frustum, int idx, int id) override
		{
			bool isintersect = false;
			for (auto& p : frustum._Planes)
			{
				float dis = p.Distance(_Center);
				if (dis > _Radius)
				{
					return BoundCheckResult::Outside;
				}
				else if (dis > -_Radius)
				{
					isintersect = true;
				}
			}
			if (isintersect)
			{
				return BoundCheckResult::Intersect;
			}
			else
			{
				return BoundCheckResult::Inside;
			}
		}
	};
}