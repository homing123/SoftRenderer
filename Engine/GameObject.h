#pragma once

#include "EngineUtil.h"


namespace HM
{
	class GameObject
	{
	public:
		GameObject() = default;
		GameObject(const std::size_t& name) {
			_Name = name;
			_Transform = Transform();
		}
		Transform& GetTransform() { return _Transform; }
		const Transform& GetTransform() const { return _Transform; }
		void SetMesh(const std::size_t meshKey) {
			_MeshKey = meshKey;

		}
		const std::size_t& GetMeshKey() const 
		{
			return _MeshKey;
		}
		void SetTexture(const std::size_t& textureKey) { 
			_TextureKey = textureKey;
		
		}
		const std::size_t& GetTextureKey() const {
			int a = 0;

			return _TextureKey;

		}
		std::size_t& GetName() {
			return _Name; 

		}

		const bool isVisible() const {	return _Active;}
		void SetActive(const bool& active) {
			_Active = active;
		}


	private:
		bool _Active = true;
		Transform _Transform;
		std::size_t _Name;
		std::size_t _MeshKey;
		std::size_t _TextureKey;
	};
}