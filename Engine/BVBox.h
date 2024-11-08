#pragma once

#include "EngineUtil.h"
namespace HM
{
	struct BVBox : BoundingVolume
	{
		BVBox(Vector3* vertices) 
		{
			_BVKind = BoundingVolumeKind::Box;
			int size = _msize(vertices) / sizeof(Vector3);
			for (int i = 0; i < size; i++)
			{
				Vector3 v = vertices[i];
				if (_Min.X > v.X)
				{
					_Min.X = v.X;
				}
				if (_Min.Y > v.Y)
				{
					_Min.Y = v.Y;
				}
				if (_Min.Z > v.Z)
				{
					_Min.Z = v.Z;
				}

				if (_Max.X < v.X)
				{
					_Max.X = v.X;
				}
				if (_Max.Y < v.Y)
				{
					_Max.Y = v.Y;
				}
				if (_Max.Z < v.Z)
				{
					_Max.Z = v.Z;
				}
			}
		}

		Vector3 _Min = Vector3::Zero;
		Vector3 _Max = Vector3::Zero;

		BoundCheckResult BoundCheck(const Frustum& frustum, int idx, int id) override
		{
			bool isintersect = false;
			int curidx = 0;
			for (auto& p : frustum._Planes)
			{
				Vector3 outPos, innerPos;
				if (p.Normal.X >= 0) //X 가 바깥면이 오른쪽일때 InnerPos.x = min.x임
				{
					innerPos.X = _Min.X; 
					outPos.X = _Max.X;
				}
				else
				{
					innerPos.X = _Max.X;
					outPos.X = _Min.X;
				}
				
				if (p.Normal.Y >= 0)
				{
					innerPos.Y = _Min.Y;
					outPos.Y = _Max.Y;
				}
				else
				{
					innerPos.Y = _Max.Y;
					outPos.Y = _Min.Y;
				}

				if (p.Normal.Z >= 0)
				{
					innerPos.Z = _Min.Z;
					outPos.Z = _Max.Z;
				}
				else
				{
					innerPos.Z = _Max.Z;
					outPos.Z = _Min.Z;
				}

				float inDis = p.Distance(innerPos);
				float outDis = p.Distance(outPos);

				if (inDis > 0)
				{
					return BoundCheckResult::Outside;
				}
				else if (inDis <= 0 && outDis >= 0)
				{
					isintersect = true;
				}
				curidx++;
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

		virtual void GetBoxInfo(Vector3& out_Min, Vector3& out_Max)
		{
			out_Min = _Min;
			out_Max = _Max;
		}
	};
}