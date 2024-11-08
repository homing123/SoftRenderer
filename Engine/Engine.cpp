#include "EngineLib.h"
#include "Resource.h"

Engine* Engine::PInstance = NULL;

static map<ControlMode, string> ControlModeToString =
{
	{MoveCam, "MoveCam"},
	{RotCam, "RotCam"},
	{InfoCam, "InfoCam"},
	{MoveObj, "MoveObj"},
	{RotObj, "RotObj"},
	{ScaleObj, "ScaleObj"},
	{ObjectCreate, "ObjectCreate"},
	{FrameTimeLog, "FrameTimeLog"},
};

static const std::string PlayerGo("Player");
const int TextCount = 20;

const float CamMoveSpeed = 5.f;
const float CamRotSpeed = 20.f;
const float CamScaleSpeed = 1.f;
const float ObjMoveSpeed = 5.f;
const float ObjRotSpeed = 20.f;
const float ObjScaleSpeed = 1.f;
float CurTime = 1.f;

extern void CU_Init(int totalMeshCount, int totalTextureCount, int totalVertexCount, int totalIndexCount, int totalTexBufferCount);
extern int CU_AddTexture(Texture* pTexture);
extern int CU_AddMesh( Mesh* pMesh);


Engine::Engine(const ScreenPoint& screenSize, const HWND& hWnd) 
{
	pInput = new InputManager();
	pRenderer = new SoftRenderer(screenSize, hWnd);
	_MainCam.SetViewPortSize(screenSize);
	_isInit = true;
	pThreadPool = new ThreadPool();
}
void Engine::ChangeScreenSize(const ScreenPoint& screenSize)
{
	if (_isInit == true) 
	{
		pRenderer->SetScreenSize(screenSize);
		_MainCam.SetViewPortSize(screenSize);
	}
}
GameObject& Engine::CreateGameObject(const std::string& name, const size_t& meshKey, const size_t& textureKey) 
{
	GameObject* obj = new GameObject(std::hash<std::string>()(name));
	_GameObjectes.push_back(std::unique_ptr<GameObject>(obj));
	obj->SetMesh(meshKey);
	obj->SetTexture(textureKey);
	return *_GameObjectes[_GameObjectes.size() - 1].get();
}
Mesh& Engine::CreateMesh(const std::size_t& meshKey, Vector3* vertices, Vector2* uvs, int* indcies, const BoundingVolumeKind& bvkind)
{
	Mesh* mesh = new Mesh(vertices, uvs, indcies, bvkind);
	_Meshes[meshKey] = std::unique_ptr<Mesh>(mesh);
	return *mesh;
}

void Engine::LoadTexture(const std::size_t textureKey, const std::string& fileName) 
{
	wstring widestr = wstring(fileName.begin(), fileName.end());
	const wchar_t* wcharName = widestr.c_str();

	Texture* texture = new Texture(wcharName);
	std::unique_ptr<Texture> unique(texture);

	_Textures[textureKey] = std::move(unique);
}
void Engine::AddText(const Text& text)
{
	_Texts.push_back(text);
}
Text& Engine::GetText(const int idx)
{
	return _Texts[idx];
}
void Engine::RemoveText(const int idx)
{
	_Texts.erase(_Texts.begin() + idx);
}

void Engine::LoadAllTexturesInPath()
{
	//GPT 코드
	// std::wstring을 사용하여 문자열 결합

	GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	std::wstring fullPath = TextureFolderPath + L"*.png";
	WIN32_FIND_DATA findFileData;
	HANDLE hFind = FindFirstFile(fullPath.c_str(), &findFileData);

	if (hFind == INVALID_HANDLE_VALUE)
	{
		//std::wcerr << L"Failed to find files in directory." << std::endl;
		return;
	}
	do
	{
		const std::wstring fileName = findFileData.cFileName;
		std::wstring fullPath = TextureFolderPath + fileName;
		std::size_t hash = std::hash<std::wstring>{}(fullPath);
		_Textures[hash] = std::make_unique<Texture>(fullPath);
	}
	while (FindNextFile(hFind, &findFileData) != 0);
	FindClose(hFind);

	GdiplusShutdown(gdiplusToken);
}
bool Engine::LoadResource() 
{
	LoadAllTexturesInPath();
	CreateMesh(QuadMeshKey, QuadVertices, QuadUVs, QuadIndices, QuadBVKind);
	CreateMesh(PlaneMeshKey, PlaneVertices, PlaneUVs, PlaneIndices, PlaneBVKind);
	CreateMesh(TestTriangleMeshKey, TestTriangleVertices, TestTriangleUVs, TestTriangleIndices, TestTriangleBVKind);
	int textureCount = 2;
	int meshCount = 3;
	int vertexCount = 0;
	int indexCount = 0;
	int texBufferSize = 0;

	Texture* pTex_FaceCube = _Textures[TexKey_FaceCube].get();
	texBufferSize += pTex_FaceCube->GetSize();
	Texture* pTex_RGBGrid = _Textures[TexKey_RGBGrid].get();
	texBufferSize += pTex_RGBGrid->GetSize();

	Mesh* pQuadMesh = _Meshes[QuadMeshKey].get();
	Mesh* pPlaneMesh = _Meshes[PlaneMeshKey].get();
	Mesh* pTestTriangleMesh = _Meshes[TestTriangleMeshKey].get();

	vertexCount += pQuadMesh->VertexCount();
	vertexCount += pPlaneMesh->VertexCount();
	vertexCount += pTestTriangleMesh->VertexCount();

	indexCount += pQuadMesh->IndexCount();
	indexCount += pPlaneMesh->IndexCount();
	indexCount += pTestTriangleMesh->IndexCount();

	CU_Init(meshCount, textureCount, vertexCount, indexCount, texBufferSize);

	_TextureIdx[TexKey_FaceCube] = CU_AddTexture(pTex_FaceCube);
	_TextureIdx[TexKey_RGBGrid] = CU_AddTexture(pTex_RGBGrid);

	_MeshIdx[QuadMeshKey] = CU_AddMesh(_Meshes[QuadMeshKey].get());
	_MeshIdx[PlaneMeshKey] = CU_AddMesh(_Meshes[PlaneMeshKey].get());
	_MeshIdx[TestTriangleMeshKey] = CU_AddMesh(_Meshes[TestTriangleMeshKey].get());


	return true;
}
void Engine::LoadScene()
{
	
}
//update, 드로우 메쉬 드로우, 칠하기 전 세팅시간, 칠하는 시간
void Engine::Start()
{
	Transform& T_Cam = _MainCam.GetTransform();
	Vector3 CamPos(0.f, 0.f, -10.f);
	Vector3 CamLookPos(0.f, 0.f, 0.f);
	T_Cam.SetPosition(CamPos);
	T_Cam.LookAt(CamLookPos);

	for (int i = 0; i < TextCount; i++)
	{
		AddText(Text("", Vector2(500, 20 +  i * 20), Color32::Black));
	}


	for (int i = 0; i < 150; i++)
	{
		GameObject& go = CreateGameObject(PlayerGo + to_string(i));

		Transform& t_obj = go.GetTransform();
		t_obj.SetPosition(Vector3(Math::Randomf(-100.f, 100.f), Math::Randomf(-100.f, 100.f), Math::Randomf(0.f, 100.f)));
		t_obj.SetScale(Vector3(Math::Randomf(3.f, 20.f), Math::Randomf(3.f, 20.f), Math::Randomf(3.f, 20.f)));
		t_obj.AddRotationY(Math::Randomf(0.f, 360.f));
	}


	//GameObject& go = CreateGameObject(PlayerGo + to_string(0), QuadMeshKey, TexKey_RGBGrid);
	//GameObject& go = CreateGameObject(PlayerGo + to_string(0), PlaneMeshKey, TexKey_RGBGrid);

	//Transform& t_obj = go.GetTransform();
	//t_obj.SetPosition(Vector3(0,0,12));
	//t_obj.SetScale(Vector3(10,10,10));
	//t_obj.AddRotationY(180);
	//t_obj.AddRotationX(90);

//	//t_obj.AddRotationX(67);
//	//t_obj.AddRotationY(22);
}

bool isCPU = false;
int addvalue(int test)
{
	return test + 5;
}
void Engine::Update() 
{
	pInput->UpdateState();
	Transform& T_Cam = Engine::PInstance->GetMainCam().GetTransform();
	if (_ElapsedTime > CurTime)
	{
		CurTime += 1;
		_ITime++;
	}
	//T_Cam.AddPosition(Vector3(0.f, 0.f, 0.01f));

	if (pInput->isKeyDown(Key::Arrow_Right))
	{
		
	}

	Control();
	WriteTextInfo();
}
void Engine::Cycle()
{
	_DeltaTime = TimeUtil::GetDeltaTime();
	_ElapsedTime += _DeltaTime;

	_TimeLog.Reset();

	float startTime = clock();
	Update();
	_TimeLog._UpdateTime = clock() - startTime;

	startTime = clock();
	pRenderer->RenderGPU();

	_TimeLog._RenderTime = clock() - startTime;

	pInput->Loging();

	TimeLogging();
}


void Engine::Control()
{
	InputManager input = Engine::PInstance->GetInput();

	if (input.isKeyDown(Key::PG_Up))
	{
		_ModeSelect = true;
	}
	else if (input.isKeyDown(Key::PG_Down))
	{
		_ModeSelect = false;
	}

	if (_ModeSelect)
	{
		if (input.isKeyDown(Key::Arrow_Right))
		{
			if (_ControlMode == FrameTimeLog)
			{
				_ControlMode = MoveCam;
			}
			else
			{
				_ControlMode = static_cast<ControlMode>(static_cast<int>(_ControlMode) + 1);
			}
		}
		if (input.isKeyDown(Key::Arrow_Left))
		{
			if (_ControlMode == MoveCam)
			{
				_ControlMode = FrameTimeLog;
			}
			else
			{
				_ControlMode = static_cast<ControlMode>(static_cast<int>(_ControlMode) - 1);
			}
		}
	}
	else
	{
		int objectCount = GetObjCount();
		switch (_ControlMode)
		{
		case ControlMode::MoveObj:
		case ControlMode::RotObj:
		case ControlMode::ScaleObj:
			if (objectCount == 0)
			{
				return;
			}
			if (input.isKeyDown(Key::Arrow_Right))
			{
				_ControlObjIdx++;
				if (_ControlObjIdx > objectCount - 1)
				{
					_ControlObjIdx = 0;
				}
			}
			else if (input.isKeyDown(Key::Arrow_Left))
			{
				_ControlObjIdx--;
				if (_ControlObjIdx < 0)
				{
					_ControlObjIdx = objectCount - 1;
				}
			}
			break;
		}
		Transform& TCam = _MainCam.GetTransform();
		GameObject* pObj = GetpGameObject(PlayerGo + to_string(0));
		Transform& TObj = pObj->GetTransform();

		switch (_ControlMode)
		{
		case ControlMode::MoveCam:
		case ControlMode::RotCam:
		case ControlMode::InfoCam:
		case ControlMode::MoveObj:
		case ControlMode::RotObj:
		case ControlMode::ScaleObj:
			if (input.isKeyDown(Key::X))
			{
				_ControlAxis = Axis::X;
			}
			else if (input.isKeyDown(Key::Y))
			{
				_ControlAxis = Axis::Y;
			}
			else if (input.isKeyDown(Key::Z))
			{
				_ControlAxis = Axis::Z;
			}
			break;
		}

		

		Vector3 vt3 = Vector3::Zero;

		switch (_ControlMode)
		{
		case ControlMode::MoveCam:
			switch (_ControlAxis)
			{
			case Axis::X:
				vt3.X = CamMoveSpeed * _DeltaTime;
				break;
			case Axis::Y:
				vt3.Y = CamMoveSpeed * _DeltaTime;
				break;
			case Axis::Z:
				vt3.Z = CamMoveSpeed * _DeltaTime;
				break;
			}
			if (input.isKey(Key::Arrow_Up))
			{
				TCam.AddPosition(vt3);
			}
			else if (input.isKey(Key::Arrow_Down))
			{
				TCam.AddPosition(vt3 * -1);
			}
			break;
		case ControlMode::RotCam:
			switch (_ControlAxis)
			{
			case Axis::X:
				if (input.isKey(Key::Arrow_Up))
				{
					TCam.AddRotationX(CamRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TCam.AddRotationX(-CamRotSpeed * _DeltaTime);
				}
				break;
			case Axis::Y:
				if (input.isKey(Key::Arrow_Up))
				{
					TCam.AddRotationY(CamRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TCam.AddRotationY(-CamRotSpeed * _DeltaTime);
				}
				break;
			case Axis::Z:
				if (input.isKey(Key::Arrow_Up))
				{
					TCam.AddRotationZ(CamRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TCam.AddRotationZ(-CamRotSpeed * _DeltaTime);
				}
				break;
			}
			
			break;
		case ControlMode::InfoCam:
			if (input.isKeyDown(Key::Arrow_Down))
			{
				ChangeScreenSize(ScreenPoint(900, 1000));
			}
			break;
		case ControlMode::MoveObj:
			switch (_ControlAxis)
			{
			case Axis::X:
				vt3.X = ObjMoveSpeed * _DeltaTime;
				break;
			case Axis::Y:
				vt3.Y = ObjMoveSpeed * _DeltaTime;
				break;
			case Axis::Z:
				vt3.Z = ObjMoveSpeed * _DeltaTime;
				break;
			}
			if (input.isKey(Key::Arrow_Up))
			{
				TObj.AddPosition(vt3);
			}
			else if (input.isKey(Key::Arrow_Down))
			{
				TObj.AddPosition(vt3 * -1);
			}
			break;
		case ControlMode::RotObj:
			switch (_ControlAxis)
			{
			case Axis::X:
				if (input.isKey(Key::Arrow_Up))
				{
					TObj.AddRotationX(ObjRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TObj.AddRotationX(-ObjRotSpeed * _DeltaTime);
				}
				break;
			case Axis::Y:
				if (input.isKey(Key::Arrow_Up))
				{
					TObj.AddRotationY(ObjRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TObj.AddRotationY(-ObjRotSpeed * _DeltaTime);
				}
				break;
			case Axis::Z:
				if (input.isKey(Key::Arrow_Up))
				{
					TObj.AddRotationZ(ObjRotSpeed * _DeltaTime);
				}
				else if (input.isKey(Key::Arrow_Down))
				{
					TObj.AddRotationZ(-ObjRotSpeed * _DeltaTime);
				}
				break;
			}

			break;
		case ControlMode::ScaleObj:
			switch (_ControlAxis)
			{
			case Axis::X:
				vt3.X = ObjScaleSpeed * _DeltaTime;
				break;
			case Axis::Y:
				vt3.Y = ObjScaleSpeed * _DeltaTime;
				break;
			case Axis::Z:
				vt3.Z = ObjScaleSpeed * _DeltaTime;
				break;
			}
			if (input.isKey(Key::Arrow_Up))
			{
				TObj.AddScale(vt3);
			}
			else if (input.isKey(Key::Arrow_Down))
			{
				TObj.AddScale(vt3 * -1);
			}
			break;
		case ControlMode::ObjectCreate:
			if (input.isKeyDown(Key::Arrow_Up))
			{
				GameObject& go = CreateGameObject(PlayerGo + to_string(objectCount));
				Transform& t_obj = go.GetTransform();
				t_obj.SetPosition(Vector3(Math::Randomf(-100.f, 100.f), Math::Randomf(-100.f, 100.f), Math::Randomf(0.f, 100.f)));
				t_obj.SetScale(Vector3(Math::Randomf(3.f, 20.f), Math::Randomf(3.f, 20.f), Math::Randomf(3.f, 20.f)));
				t_obj.AddRotationY(Math::Randomf(0.f, 360.f));
			}
			else if (input.isKeyDown(Key::Arrow_Down))
			{
				
			}
			break;
		}
	}
}

void Engine::WriteTextInfo() 
{
	Text& text0 = GetText(0);
	text0.ChangeString("ControlMode : " + ControlModeToString[_ControlMode]);
	Text& text1 = Engine::PInstance->GetText(1);
	text1.ChangeString("ITime = " + std::to_string(Engine::PInstance->_ITime) + " FPS = " + std::to_string(1 / Engine::PInstance->GetDeltaTime()));
	Text& text2 = GetText(2);
	if (_ModeSelect)
	{
		text2.ChangeString("ControlMode");
	}
	else
	{

		int objectCount = GetObjCount();
		switch (_ControlMode) {
		case ControlMode::MoveObj:
		case ControlMode::ScaleObj:
		case ControlMode::RotObj:
			if (objectCount == 0)
			{
				text2.ChangeString("Active Object Count : 0 ");
				return;
			}
			break;
		}

		Transform& TCam = _MainCam.GetTransform();
		GameObject* pObj = GetpGameObject(PlayerGo + to_string(0));
		Transform& TObj = pObj->GetTransform();
		Vector3 vt3;
		switch (_ControlMode)
		{
		case ControlMode::MoveCam:
			vt3 = TCam.GetPos();
			text2.ChangeString("CamPos X : " + std::to_string(vt3.X) + " Y : " + std::to_string(vt3.Y) + " Z : " + std::to_string(vt3.Z));
			break;
		case ControlMode::RotCam:
			vt3 = TCam.GetEuler();
			text2.ChangeString("CamRot X : " + std::to_string(vt3.X) + " Y : " + std::to_string(vt3.Y) + " Z : " + std::to_string(vt3.Z));
			break;
		case ControlMode::InfoCam:
		{
			float fov = _MainCam.GetFOV();
			float _near = 0, _far = 0;
			_MainCam.GetNearFar(_near, _far);
			text2.ChangeString("CamFOV : " + std::to_string(fov) + " CamNear : " + std::to_string(_near) + " Z : " + std::to_string(_far));
		}
		break;
		case ControlMode::MoveObj:
			vt3 = TObj.GetPos();
			text2.ChangeString("Obj " + std::to_string(0) + " Pos X : " + std::to_string(vt3.X) + " Y : " + std::to_string(vt3.Y) + " Z : " + std::to_string(vt3.Z));
			break;
		case ControlMode::RotObj:
			vt3 = TObj.GetEuler();
			text2.ChangeString("Obj " + std::to_string(0) + " Rot X : " + std::to_string(vt3.X) + " Y : " + std::to_string(vt3.Y) + " Z : " + std::to_string(vt3.Z));
			break;
		case ControlMode::ScaleObj:
			vt3 = TObj.GetScale();
			text2.ChangeString("Obj " + std::to_string(0) + " Scale X : " + std::to_string(vt3.X) + " Y : " + std::to_string(vt3.Y) + " Z : " + std::to_string(vt3.Z));
			break;
		case ControlMode::ObjectCreate:
			text2.ChangeString("Object Count : " + std::to_string(objectCount));
			break;
		}
	}
}

void Engine::TimeLogging()
{
	if (_ControlMode == ControlMode::FrameTimeLog)
	{
		GetText(2).ChangeString("UpdateTime : " + to_string(_TimeLog._UpdateTime));
		GetText(3).ChangeString("RenderTime : " + to_string(_TimeLog._RenderTime));
		GetText(4).ChangeString("CU_BG_Depth_CamInfoSetting : " + to_string(_TimeLog.CU_BG_Depth_CamInfoSetting));
		GetText(5).ChangeString("CU_DrawCallSetting : " + to_string(_TimeLog.CU_DrawCallSetting));
		GetText(6).ChangeString("CU_BoundCheckBox : " + to_string(_TimeLog.CU_BoundCheckBox));
		GetText(7).ChangeString("CU_VertexArraySetting : " + to_string(_TimeLog.CU_VertexArraySetting));
		GetText(8).ChangeString("CU_Culling_ScanLine_SetBuffer : " + to_string(_TimeLog.CU_Culling_ScanLine_SetBuffer));
		GetText(9).ChangeString("CU_SetBuffer : " + to_string(_TimeLog.CU_SetBuffer));
		GetText(10).ChangeString("CU_BufferCopy : " + to_string(_TimeLog.CU_BufferCopy));
		GetText(11).ChangeString("CU_Test : " + to_string(_TimeLog.CU_Test));
		GetText(12).ChangeString("CU_Test1 : " + to_string(_TimeLog.CU_Test1));

	}
	else
	{
		for (int i = 2; i < TextCount; i++)
		{
			GetText(i).ChangeString("");
		}
	}
}


void Engine::CallThreadPool(const int idxCount, std::function<void(int, int, int)> func)
{
	int threadCount = ThreadCount + 1;
	int Count_Per_Thread = idxCount / threadCount;
	int remainCount = idxCount % threadCount;

	int curIdx = Count_Per_Thread;
	int curCount, endIdx;
	for (int i = 1; i < threadCount; i++)
	{
		curCount = Count_Per_Thread;
		if (remainCount > 0)
		{
			curCount += 1;
			remainCount -= 1;
		}

		endIdx = curIdx + curCount;
		std::function<void()> job = [this, i, curIdx, endIdx, func]() {func(i, curIdx, endIdx); };
		pThreadPool->EnqueueJob(job);

		curIdx = endIdx;
	}

	func(0, 0, Count_Per_Thread);
	pThreadPool->WaitForJoin();
}