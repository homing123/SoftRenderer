#include <EngineLib.h>
#include <cuda.h>
#include <cufft.h>
#include <cuda_runtime.h>

struct d_ScreenPoint
{
public:
	__device__ d_ScreenPoint() = default;

	int X, Y;    
};
struct d_Vector2
{
public:
	__device__ d_Vector2() = default;
	__device__ d_Vector2(float _x, float _y)
	{
		X = _x;
		Y = _y;
	}
	float X, Y;
	__device__ d_Vector2 operator *(const float value)
	{
		return d_Vector2(X * value, Y * value);
	}
	__device__ d_Vector2 operator +(const d_Vector2& vt2)
	{
		return d_Vector2(X + vt2.X, Y + vt2.Y);
	}
};

struct d_Vector3
{
public:
	__device__ d_Vector3() = default;
	__device__ d_Vector3(const float x, const float y, const float z)
	{
		X = x;
		Y = y;
		Z = z;
	}
	float X, Y, Z;
	__device__ d_Vector3 operator *(const float value)
	{
		return d_Vector3(X * value, Y * value, Z * value);
	}
	__device__ d_Vector3 operator -(const d_Vector3& vt3)
	{
		return d_Vector3(X - vt3.X, Y - vt3.Y, Z - vt3.Z);
	}
	__device__ float Dot(const d_Vector3& vt3)
	{
		return X * vt3.X + Y * vt3.Y + Z * vt3.Z;
	}
	__device__ d_Vector3 CrossProduct(const d_Vector3& vt3)
	{
		return d_Vector3(Y * vt3.Z - Z * vt3.Y, Z * vt3.X - X * vt3.Z, X * vt3.Y - Y * vt3.X);
	}
};
struct d_Vector4
{
public:
	__device__ d_Vector4() = default;
	__device__ d_Vector4(const float x, const float y, const float z, const float w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	float X, Y, Z, W;

	__device__ d_Vector4 operator *(const float value)
	{
		return d_Vector4(X * value, Y * value, Z * value, W * value);
	}
};

struct d_Matrix4x4
{
public:
	__device__ d_Matrix4x4() = default;

	float Ele[16];

	__device__ d_Vector4 operator *(const d_Vector4& vt4)
	{
		return d_Vector4(Ele[0] * vt4.X + Ele[1] * vt4.Y + Ele[2] * vt4.Z + Ele[3] * vt4.W,
			Ele[4] * vt4.X + Ele[5] * vt4.Y + Ele[6] * vt4.Z + Ele[7] * vt4.W,
			Ele[8] * vt4.X + Ele[9] * vt4.Y + Ele[10] * vt4.Z + Ele[11] * vt4.W,
			Ele[12] * vt4.X + Ele[13] * vt4.Y + Ele[14] * vt4.Z + Ele[15] * vt4.W);
	}

};

struct d_LinearColor
{
public:
	__device__ d_LinearColor() = default;

	float R,G,B,A;
};

struct d_Vertex
{
public:
	__device__ d_Vertex() = default;

	d_Vector4 Pos;
	d_Vector2 UV;
};

struct d_Vt2Idx
{
public:
	__device__ d_Vt2Idx() = default;

	d_Vector2 Pos;
	int Idx;
};
struct d_Color32
{
public:
	__device__ d_Color32() = default;
	unsigned char B, G, R, A;
};


enum d_CullType
{
	Off,
	Front,
	Back
};

struct d_RenderState
{
public:
	__device__ d_RenderState() = default;
	d_CullType CullType = d_CullType::Back;
};

//struct d_DrawCall
//{
//public :
//	__device__ d_DrawCall() = default;
//	__device__ ~d_DrawCall()
//	{
//		cudaFree(arr_MeshIdx);
//		cudaFree(arr_LocalToWorldMat);
//	}
//	void Setting(int materialIdx, vector<GameObject*> v_pGo_Visible)
//	{
//		int goCount = v_pGo_Visible.size();
//		cudaMemcpy(&GoCount, &goCount, sizeof(int), cudaMemcpyHostToDevice);
//		cudaMemcpy(&MaterialIdx, &materialIdx, sizeof(int), cudaMemcpyHostToDevice);
//
//		int* arr_meshidx = new int[goCount];
//		d_Matrix4x4* arr_mat = new d_Matrix4x4[goCount];
//
//		cudaMalloc(&arr_MeshIdx, sizeof(int) * goCount);
//		cudaMalloc(&arr_LocalToWorldMat, sizeof(d_Matrix4x4) * goCount);
//
//		cudaMemcpy(&arr_MeshIdx, &arr_meshidx, sizeof(int) * goCount, cudaMemcpyHostToDevice);
//		cudaMemcpy(&arr_LocalToWorldMat, &arr_mat, sizeof(d_Matrix4x4) * goCount, cudaMemcpyHostToDevice);
//
//		delete[] arr_meshidx;
//		delete[] arr_mat;
//	}
//	int MaterialIdx;
//	int GoCount;
//	int* arr_MeshIdx;
//	d_Matrix4x4* arr_LocalToWorldMat;
//};
//struct d_Mesh
//{
//public:
//	__device__ d_Mesh() = default;
//	__device__ ~d_Mesh()
//	{
//		//d_Mesh free 할때 자동으로 불리는지 확인
//		cudaFree(Vertices);
//		cudaFree(UVs);
//		cudaFree(Indices);
//	}
//	void Setting(Mesh * pMesh)
//	{
//		int vertexCount = pMesh->VertexCount();
//		int indexCount = pMesh->IndexCount();
//
//		cudaMemcpy(&VertexCount, &vertexCount, sizeof(int), cudaMemcpyHostToDevice);
//		cudaMemcpy(&IndexCount, &indexCount, sizeof(int), cudaMemcpyHostToDevice);
//
//		cudaMalloc((void**)&Vertices, sizeof(d_Vector3) * vertexCount);
//		cudaMalloc((void**)&UVs, sizeof(d_Vector2) * vertexCount);
//		cudaMalloc((void**)&Indices, sizeof(int) * indexCount);
//
//		cudaMemcpy(Vertices, pMesh->GetVertices(), sizeof(d_Vector3) * vertexCount, cudaMemcpyHostToDevice);
//		cudaMemcpy(UVs, pMesh->GetUVs(), sizeof(d_Vector2) * vertexCount, cudaMemcpyHostToDevice);
//		cudaMemcpy(Indices, pMesh->GetIndices(), sizeof(int) * indexCount, cudaMemcpyHostToDevice);
//	}
//	__device__ const d_Vector3* GetVertices() { return Vertices; }
//	__device__ const d_Vector2* GetUVs() { return UVs; }
//	__device__ const int* GetIndices() { return Indices; }
//	int VertexCount;
//	int IndexCount;
//private:
//	d_Vector3* Vertices;
//	d_Vector2* UVs;
//	int* Indices;
//};
//struct d_Texture
//{
//public:
//	__device__ d_Texture() = default;
//	__device__ ~d_Texture()
//	{
//		cudaFree(Buffer);
//	}
//	void Setting(Texture* pTexture)
//	{
//		int width = pTexture->GetWidth();
//		int height = pTexture->GetHeight();
//		int bufferSize = pTexture->GetSize();
//
//		cudaMemcpy(&Width, &width, sizeof(int), cudaMemcpyHostToDevice);
//		cudaMemcpy(&Height, &height, sizeof(int), cudaMemcpyHostToDevice);
//
//		cudaMalloc(&Buffer, sizeof(d_Color32) * bufferSize);
//		cudaMemcpy(&Buffer, pTexture->GetBuffer(), sizeof(d_Color32) * bufferSize, cudaMemcpyHostToDevice);
//	}
//
//	__device__ const d_Color32* GetBuffer() { return Buffer; }
//	__device__ const int BufferSize() { return Width * Height; }
//	int Width;
//	int Height;
//private:
//	d_Color32* Buffer;
//};