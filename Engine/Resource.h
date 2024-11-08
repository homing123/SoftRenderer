#pragma once
#include "EngineUtil.h"

//¿Þ¼ÕÁÂÇ¥°è
#pragma Mesh
#pragma region Quad
const static std::size_t QuadMeshKey = std::hash<std::string>()("SM_Quad");
constexpr static float QuadHalfSize = 0.5f;

static Vector3* QuadVertices = new Vector3[24]{
	//-y
	Vector3(-QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	Vector3(-QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	//+y	
	Vector3(-QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	Vector3(-QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	//+z
	Vector3(QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(-QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(-QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	//+x
	Vector3(QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	//-z
	Vector3(-QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
	Vector3(-QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	Vector3(QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
	//-x
	Vector3(-QuadHalfSize,-QuadHalfSize,QuadHalfSize),
	Vector3(-QuadHalfSize,QuadHalfSize,QuadHalfSize),
	Vector3(-QuadHalfSize,QuadHalfSize,-QuadHalfSize),
	Vector3(-QuadHalfSize,-QuadHalfSize,-QuadHalfSize),
};
static Vector2* QuadUVs = new Vector2[24] {
	//-y
	Vector2(0.5f,0.f),
	Vector2(0.25f,0.f),
	Vector2(0.25f,0.25f),
	Vector2(0.5f,0.25f),
	//+y
	Vector2(0.5f,0.75f),
	Vector2(0.5f,0.5f),
	Vector2(0.25f,0.5f),
	Vector2(0.25f,0.75f),
	//+z
	Vector2(0.25f,0.25f),
	Vector2(0.25f,0.5f),
	Vector2(0.5f,0.5f),
	Vector2(0.5f,0.25f),
	//+x
	Vector2(0.f,0.25f),
	Vector2(0.f,0.5f),
	Vector2(0.25f,0.5f),
	Vector2(0.25f,0.25f),
	//-z
	Vector2(0.75f,0.25f),
	Vector2(0.75f,0.5f),
	Vector2(1.f,0.5f),
	Vector2(1.f,0.25f),
	//-x
	Vector2(0.5f,0.25f),
	Vector2(0.5f,0.5f),
	Vector2(0.75f,0.5f),
	Vector2(0.75f,0.25f),
};

static int* QuadIndices = new int[36]
{
		0,1,2,0,2,3, //-y
		4,5,6,4,6,7, //+y
		8,9,10,8,10,11, //+z
		12,13,14,12,14,15, //+x
		16,17,18,16,18,19, //-z
		20,21,22,20,22,23 //-x
};

const static BoundingVolumeKind QuadBVKind = BoundingVolumeKind::Box;

#pragma endregion
#pragma region Plane
const static std::size_t PlaneMeshKey = std::hash<std::string>()("SM_Plane");
constexpr static float PlaneHalfSize = 0.5f;

static Vector3* PlaneVertices = new Vector3[4] {

	//+y	
	Vector3(-PlaneHalfSize,0.f,-PlaneHalfSize),
	Vector3(-PlaneHalfSize,0.f,PlaneHalfSize),
	Vector3(PlaneHalfSize,0.f,PlaneHalfSize),
	Vector3(PlaneHalfSize,0.f,-PlaneHalfSize),

};
static Vector2* PlaneUVs = new Vector2[4]{

	//+y
	Vector2(0.f,0.f),
	Vector2(0.f,1.f),
	Vector2(1.f,1.f),
	Vector2(1.f,0.f),

};

static int* PlaneIndices = new int[6]
{
		0,1,2,0,2,3, //+y
};

const static BoundingVolumeKind PlaneBVKind = BoundingVolumeKind::Box;

#pragma endregion
#pragma region TestTriangle

const static std::size_t TestTriangleMeshKey = std::hash<std::string>()("SM_TestTriangle");
constexpr static float TestTriangleHalfSize = 0.5f;

static Vector3* TestTriangleVertices = new Vector3 [3]
{
	//+y	
	Vector3(-TestTriangleHalfSize,0.f,-TestTriangleHalfSize),
	Vector3(-TestTriangleHalfSize,0.f,TestTriangleHalfSize),
	Vector3(TestTriangleHalfSize,0.f,-TestTriangleHalfSize),

};
static Vector2* TestTriangleUVs = new Vector2 [3]
{
	//+y
	Vector2(0.f,0.f),
	Vector2(0.f,1.f),
	Vector2(1.f,0.f),
};

static int* TestTriangleIndices = new int[3]
{
		0,1,2, //+y
};

const static BoundingVolumeKind TestTriangleBVKind = BoundingVolumeKind::Box;

#pragma endregion
#pragma endregion