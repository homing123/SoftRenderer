#pragma once
#include "EngineUtil.h"

namespace HM
{
	enum class BoundCheckResult : UINT32
	{
		Outside = 0,
		Intersect,
		Inside
	};
	struct Frustum
	{
	public:
		FORCEINLINE Frustum(const Matrix4x4& matT)
		{
			Vector4 column[4] =
			{
				matT.GetColumn(0),
				matT.GetColumn(1),
				matT.GetColumn(2),
				matT.GetColumn(3)
			};

			_Planes[0] = Plane((column[3] - column[1]) * -1); // +y
			_Planes[1] = Plane((column[3] + column[1]) * -1); // -y
			_Planes[2] = Plane((column[3] - column[0]) * -1); // +x
			_Planes[3] = Plane((column[3] + column[0]) * -1); // -x
			_Planes[4] = Plane((column[3] - column[2]) * -1); // +z
			_Planes[5] = Plane((column[3] + column[2]) * -1); // -z
		}
		FORCEINLINE Frustum(Plane* planes)
		{
			_Planes[0] = planes[0];
			_Planes[1] = planes[1];
			_Planes[2] = planes[2];
			_Planes[3] = planes[3];
			_Planes[4] = planes[4];
			_Planes[5] = planes[5];
		}
		Plane _Planes[6];

		FORCEINLINE constexpr BoundCheckResult CheckBound(const Vector3& pos) const
		{
			int idx = 0;
			for (const auto& p : _Planes)
			{
				if (p.isUp(pos))
				{
					return BoundCheckResult::Outside;
				}
				else if (Math::CheckRangeSM(p.Distance(pos), 0.f))
				{
					return BoundCheckResult::Intersect;
				}
			}
			return BoundCheckResult::Inside;

		}

	};
}