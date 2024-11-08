#pragma once

#include "EngineUtil.h"


class SoftRenderer;

enum ControlMode : int
{
	MoveCam = 1,
	RotCam,
	InfoCam,
	MoveObj,
	RotObj,
	ScaleObj,
	ObjectCreate,
	FrameTimeLog, //이거 마지막으로 두기 controlmode 변경로직에 마지막이 FrameTimeLog 일때로 가정함
};
enum Axis
{
	X,
	Y,
	Z
};
namespace HM {
	class Engine
	{
	public:
		static constexpr char InvalidHashName[] = "InvalidHash";
		const std::size_t InvaildHash = std::hash<std::string>()(InvalidHashName);
		GameObject InvaildGO = GameObject(InvaildHash);
		Engine(const ScreenPoint& screenSize, const HWND& hWnd);
		GameObject& CreateGameObject(const std::string& name, const size_t& meshKey = QuadMeshKey, const size_t& textureKey = ErrorTextureKey);
		GameObject* GetpGameObject(const std::string& name)
		{
			std::size_t hash = std::hash<std::string>()(name);
			int size = _GameObjectes.size();
			for (int i = 0; i < size; i++) 
			{
				GameObject* pobj = _GameObjectes[i].get();
				if (pobj->GetName() == hash)
				{
					return pobj;
				}
			}
			return nullptr;
		}
		GameObject* GetpGameObject(const int idx)
		{
			if (idx >= 0 && idx < _GameObjectes.size())
			{
				return _GameObjectes[idx].get();
			}
			else 
			{
				return nullptr;
			}
		}


		Mesh& CreateMesh(const std::size_t& meshKey, Vector3* vertices, Vector2* uvs, int* indcies, const BoundingVolumeKind& bvkind);
		Mesh* GetpMesh(const std::size_t& meshKey)
		{
			return _Meshes.at(meshKey).get();
		}
		const int GetMeshIdx(const std::size_t& meshKey) const
		{
			return _MeshIdx.at(meshKey);
		}

		void LoadTexture(const std::size_t textureKey, const std::string& fileName);
		Texture* GetpTexture(const std::size_t& textureKey)
		{
			if (_Textures.find(textureKey) == _Textures.end()) {
				return nullptr;
			}
			else {
				return _Textures.at(textureKey).get();
			}
		}
		const int GetTextureKey (const std::size_t& textureKey) const
		{
			return _TextureIdx.at(textureKey);
		}

		const vector<GameObject*> GetVisibleGO() const
		{
			vector<GameObject*> v_pGo_Visible;
			int size = _GameObjectes.size();
			for (int i = 0; i < size; i++)
			{
				GameObject* pObj = _GameObjectes[i].get();
				if (pObj->isVisible())
				{
					v_pGo_Visible.push_back(pObj);
				}
			}
			return v_pGo_Visible;
		}

		static Engine* PInstance;
		bool LoadResource();
		void LoadScene();
		void Start();
		void Cycle();
		void AddText(const Text& text);
		Text& GetText(const int idx);
		void RemoveText(const int idx);

		InputManager& GetInput() const
		{
			return *pInput;
		}
		int GetObjCount()
		{
			return _GameObjectes.size();
		}
		const std::vector<Text> GetTexts() const
		{
			return _Texts;
		}
		const int GetObjCount() const
		{
			return _GameObjectes.size();
		}

		FORCEINLINE Camera& GetMainCam() { return _MainCam; }
		FORCEINLINE const Camera& GetMainCam() const { return _MainCam; }
		FORCEINLINE constexpr float GetDeltaTime() { return _DeltaTime; }
		
		int _ITime = 0;
		int _IdxCPU = 0;
		int _IdxGPU = 0;
		bool _ModeSelect = false;
		ControlMode _ControlMode = ControlMode::FrameTimeLog;
		Axis _ControlAxis = Axis::X;
		int _ControlObjIdx = 0;
		TimeLog _TimeLog;

		void Control();
		void WriteTextInfo();
		void TimeLogging();
		void ChangeScreenSize(const ScreenPoint& size);
		void CallThreadPool(const int idxCount, std::function<void(int, int, int)> func);
	private:
		//size_t는 정수중 양수, 0을 나타내고 보통 크기를 나타낼때 사용한다고 한다
		//unique_ptr은 객체소유권을 하나만 가지도록 할때 사용한다. 중복 delete 방지
		//unordered_map 데이터 양이 많아질수록 map보다 빠름, 키값이 비슷할수록 속도가 느려짐, c# 의 dictionary
		bool _isInit;
		std::vector<std::unique_ptr<GameObject>> _GameObjectes;
		std::unordered_map<std::size_t, std::unique_ptr<Mesh>> _Meshes;
		std::unordered_map<size_t, int> _MeshIdx;
		std::unordered_map<std::size_t, std::unique_ptr<Texture>> _Textures;
		std::unordered_map<size_t, int> _TextureIdx;

		std::vector<Text> _Texts;

		int _Frame = 60;
		float _DeltaTime = 0.f;
		float _ElapsedTime = 0.f;

		void LoadAllTexturesInPath();
		void Update();
		SoftRenderer* pRenderer;
		InputManager* pInput;
		ThreadPool* pThreadPool;
		Camera _MainCam;
	};
}