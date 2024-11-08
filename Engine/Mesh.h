#pragma once
#include "EngineUtil.h"
namespace HM
{
	class Mesh {

	public:
		Mesh() = default;
		Mesh(Vector3* vertices, Vector2* uvs, int* indcies, const BoundingVolumeKind& bvkind)
		{
			Vertices = vertices; 
			UVs = uvs;
			Indices = indcies;
			SetBoundingVolume(bvkind);
		}

		const Vector3* GetVertices() const { return Vertices; }
		const Vector2* GetUVs() const { return UVs; }
		const int* GetIndices() const { return Indices; }

		const int IndexCount() { return _msize(Indices) / sizeof(int); }
		const int VertexCount() { return _msize(Vertices) / sizeof(Vector3); }

		void SetBoundingVolume(const BoundingVolumeKind& bvkind) 
		{
			switch (bvkind)
			{
				case BoundingVolumeKind::Sphere:
					_BV = new BVSphere(Vertices);
					break;
				case BoundingVolumeKind::Box:
					_BV = new BVBox(Vertices);
					break;
			}
		}
		BoundingVolume* GetpBV() { return _BV; }


	private:
		Vector3* Vertices;
		Vector2* UVs;
		int* Indices;

		BoundingVolume* _BV;

	};
}