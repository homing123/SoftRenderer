#include "EngineLib.h"
#include "CUDA_Define.cuh"

const int h_MinThreadCount = 32;
const int h_MaxThreadCount = 512; // 1024 라고 하던데 1024하니까 진행안됐었음
const int h_MaxBlockCountX = 1024;
const int h_MaxBlockCountY = 1024;
const int h_ThreadMulBlockX = h_MaxThreadCount * h_MaxBlockCountX;
const int h_MaxTotalCount = h_MaxThreadCount * h_MaxBlockCountX * h_MaxBlockCountY;
float* h_Log = new float[100];
float* d_Log;
#pragma region 공용함수
const int MatSize = sizeof(float4) * 4;

//인터넷에서 긁어온거 안에 돌아가는 내용 알아볼 필요 있음
__device__ static float atomicMax(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fmaxf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}

__device__ static float atomicMin(float* address, float val)
{
	int* address_as_i = (int*)address;
	int old = *address_as_i, assumed;
	do {
		assumed = old;
		old = ::atomicCAS(address_as_i, assumed,
			__float_as_int(::fminf(val, __int_as_float(assumed))));
	} while (assumed != old);
	return __int_as_float(old);
}


//남는갯수 최대한 줄일방법 찾아야함
void Block_Thread_Size(const int total_Size, dim3& out_Block, int& out_Thread)
{
	if (total_Size > h_MaxTotalCount)
	{
		int a = 0;
		return;
	}
	else if (total_Size >= h_ThreadMulBlockX)
	{
		out_Block.z = 1;
		if (total_Size == h_ThreadMulBlockX)
		{
			out_Block.y = total_Size / h_ThreadMulBlockX;
		}
		else
		{
			out_Block.y = total_Size / h_ThreadMulBlockX + 1;
		}
		out_Block.x = h_MaxBlockCountX;
		out_Thread = h_MaxThreadCount;
	}
	else if (total_Size >= h_MaxThreadCount)
	{
		out_Block.z = 1;
		out_Block.y = 1;
		if (total_Size == h_MaxThreadCount)
		{
			out_Block.x = total_Size / h_MaxThreadCount;
		}
		else
		{
			out_Block.x = total_Size / h_MaxThreadCount + 1;
		}
		out_Thread = h_MaxThreadCount;
	}
	else
	{
		out_Block.z = 1;
		out_Block.y = 1;
		out_Block.x = 1;
		out_Thread = (((total_Size >> 5) + 1) << 5);
	}
}

//타일링 기법 써보기
__constant__ int MatMulMatCount;
__constant__ float4 ConstantMat[4];

__global__ void ConstantMatMulMat(d_Matrix4x4* s_m, d_Matrix4x4* d_m)
{
	//extern __shared__ float rightMat[];
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= MatMulMatCount)
	{
		return;
	}
	int matIdx = idx >> 4;
	int eleIdx = idx & 15;
	int _y = eleIdx >> 2; //warp = 32 thread, 한 워프의 _y값은 일정하다
	int _x = eleIdx & 3;
	float4 leftf4 = ConstantMat[_y];

	//tiling 미적용
	d_m[matIdx].Ele[eleIdx] = leftf4.x * s_m[matIdx].Ele[_x] + leftf4.y * s_m[matIdx].Ele[_x + 4] + leftf4.z * s_m[matIdx].Ele[_x + 8] + leftf4.w * s_m[matIdx].Ele[_x + 12];

	//tiling 적용
	/*rightMat[threadIdx.x] = s_m[matIdx].Ele[eleIdx];
	__syncthreads();
	int mulValue = (matIdx * 16) & (blockDim.x - 1);
	d_m[matIdx].Ele[eleIdx] = leftf4.x * rightMat[mulValue + _x] + leftf4.y * rightMat[mulValue + _x + 4] + leftf4.z * rightMat[mulValue + _x + 8] + leftf4.w * rightMat[mulValue + _x + 12];*/

}
cudaStream_t S_ConstantMatMulMat;
void CU_ConstantMatMulMat(int count, Matrix4x4& h_mat, d_Matrix4x4* s_m, d_Matrix4x4* d_m)
{
	cudaMemcpyToSymbolAsync(ConstantMat, &h_mat, MatSize, 0, cudaMemcpyHostToDevice, S_ConstantMatMulMat);
	int h_matmulCount = count * 16;
	cudaMemcpyToSymbolAsync(MatMulMatCount, &h_matmulCount, sizeof(int), 0, cudaMemcpyHostToDevice, S_ConstantMatMulMat);

	dim3 block;
	int thread;
	Block_Thread_Size(count * 16, block, thread);
	//tiling 미적용
	ConstantMatMulMat << <block, thread, thread * sizeof(float), S_ConstantMatMulMat >> > (s_m, d_m);

	//tiling 적용
	//ConstantMatMulMat << <block, thread,0, S_ConstantMatMulMat >> > (s_m, d_m);

	cudaStreamSynchronize(S_ConstantMatMulMat);
}
__constant__ int KoggeStoneCount_Block;

//0, 511, 1023, 1535 ...~ 값들을 누적합 해준다.
__global__ void KoggeStoneScan_blocks(int* d_res, int* d_blockAddValue)
{
	extern __shared__ int shared[];
	int idx = threadIdx.x;
	if (idx >= KoggeStoneCount_Block)
	{
		return;
	}
	if (idx == 0)
	{
		shared[idx] = 0;
	}
	else
	{
		shared[idx] = d_res[511 + 512 * (idx - 1)];
	}
	for (int stride = 1; stride < blockDim.x; stride <<= 1)
	{
		__syncthreads();
		if (threadIdx.x >= stride)
		{
			shared[threadIdx.x] += shared[threadIdx.x - stride];
		}
	}

	d_blockAddValue[idx] = shared[threadIdx.x];
}

//누적합 된 값을 더해준다.
__global__ void KoggeStoneScan_int_Final(int* d_res, int* d_blockAddValue, int count)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= count)
	{
		return;
	}
	int bIdx = gridDim.x * blockIdx.y + blockIdx.x;
	d_res[idx] += d_blockAddValue[bIdx];
}


void CU_KoggeStoneScan_Block(int* d_res, dim3 block, int thread, int count, int blockCount, cudaStream_t& stream)
{
	cudaMemcpyToSymbolAsync(KoggeStoneCount_Block, &blockCount, sizeof(int), 0, cudaMemcpyHostToDevice, stream);
	int blockthread = (((blockCount >> 5) + 1) << 5);

	int* d_AddValue;
	cudaMalloc(&d_AddValue, sizeof(int) * blockCount);
	KoggeStoneScan_blocks << <1, blockthread, blockthread * sizeof(int), stream >> > (d_res, d_AddValue);
	KoggeStoneScan_int_Final << <block, thread, 0, stream >> > (d_res, d_AddValue, count);
	cudaFree(d_AddValue);
}
#pragma endregion


int* d_TexBufferStartIdx;
int* d_TexBufferCount;

int* d_TexWidth;
int* d_TexHeight;

d_Color32* d_TexBuffer;

int* d_MeshVertexStartIdx;
int* d_MeshVertexCount;

int* d_MeshIndexStartIdx;
int* d_MeshIndexCount;

float3* d_MeshVertices;
float2* d_MeshUVs;
int* d_MeshIndices;

float3* d_BvBox_Min;
float3* d_BvBox_Max;

int h_CurUploadTextureCount = 0;
int h_CurUploadTexBufferCount = 0;

int h_CurUploadMeshCount = 0;
int h_CurUploadVertexCount = 0;
int h_CurUploadIndexCount = 0;
std::map<int, int> h_MeshTriangleCountLookUpTable;
cudaStream_t S_BGDepthInit;

void CU_Init(int totalMeshCount, int totalTextureCount, int totalVertexCount, int totalIndexCount, int totalTexBufferCount)
{
	cudaStreamCreate(&S_BGDepthInit);
	cudaStreamCreate(&S_ConstantMatMulMat);
	cudaMalloc(&d_Log, sizeof(float) * 100);

	cudaMalloc(&d_TexBufferStartIdx, sizeof(int) * totalTextureCount);
	cudaMalloc(&d_TexBufferCount, sizeof(int) * totalTextureCount);
	cudaMalloc(&d_TexWidth, sizeof(int) * totalTextureCount);
	cudaMalloc(&d_TexHeight, sizeof(int) * totalTextureCount);

	cudaMalloc(&d_TexBuffer, sizeof(d_Color32) * totalTexBufferCount);

	cudaMalloc(&d_MeshVertexStartIdx, sizeof(int) * totalMeshCount);
	cudaMalloc(&d_MeshVertexCount, sizeof(int) * totalMeshCount);
	cudaMalloc(&d_MeshIndexStartIdx, sizeof(int) * totalMeshCount);
	cudaMalloc(&d_MeshIndexCount, sizeof(int) * totalMeshCount);

	cudaMalloc(&d_MeshVertices, sizeof(float3) * totalVertexCount);
	cudaMalloc(&d_MeshUVs, sizeof(float2) * totalVertexCount);
	cudaMalloc(&d_MeshIndices, sizeof(int) * totalIndexCount);
	cudaMalloc(&d_BvBox_Min, sizeof(float3) * totalMeshCount);
	cudaMalloc(&d_BvBox_Max, sizeof(float3) * totalMeshCount);
}

int CU_AddTexture(Texture* pTexture)
{
	int bufferSize = pTexture->GetSize();
	int width = pTexture->GetWidth();
	int height = pTexture->GetHeight();
	cudaMemcpy(d_TexBufferStartIdx + h_CurUploadTextureCount, &h_CurUploadTexBufferCount, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_TexBufferCount + h_CurUploadTextureCount, &bufferSize, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_TexWidth + h_CurUploadTextureCount, &width, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_TexHeight + h_CurUploadTextureCount, &width, sizeof(int), cudaMemcpyHostToDevice);

	cudaMemcpy(d_TexBuffer + h_CurUploadTexBufferCount, pTexture->GetBuffer(), sizeof(d_Color32) * bufferSize, cudaMemcpyHostToDevice);

	h_CurUploadTextureCount++;
	h_CurUploadTexBufferCount += bufferSize;

	return h_CurUploadTextureCount - 1;
}

int CU_AddMesh(Mesh* pMesh)
{
	int vertexCount = pMesh->VertexCount();
	int indexCount = pMesh->IndexCount();

	h_MeshTriangleCountLookUpTable[h_CurUploadMeshCount] = indexCount / 3;

	cudaMemcpy(d_MeshVertexStartIdx + h_CurUploadMeshCount, &h_CurUploadVertexCount, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_MeshVertexCount + h_CurUploadMeshCount, &vertexCount, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_MeshVertices + h_CurUploadVertexCount, pMesh->GetVertices(), sizeof(float3) * vertexCount, cudaMemcpyHostToDevice);
	cudaMemcpy(d_MeshUVs + h_CurUploadVertexCount, pMesh->GetUVs(), sizeof(float2) * vertexCount, cudaMemcpyHostToDevice);

	cudaMemcpy(d_MeshIndexStartIdx + h_CurUploadMeshCount, &h_CurUploadIndexCount, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_MeshIndexCount + h_CurUploadMeshCount, &indexCount, sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(d_MeshIndices + h_CurUploadIndexCount, pMesh->GetIndices(), sizeof(int) * indexCount, cudaMemcpyHostToDevice);

	Vector3 bv_min, bv_max;
	pMesh->GetpBV()->GetBoxInfo(bv_min, bv_max);
	cudaMemcpy(d_BvBox_Max + h_CurUploadMeshCount, &bv_max, sizeof(float3), cudaMemcpyHostToDevice);
	cudaMemcpy(d_BvBox_Min + h_CurUploadMeshCount, &bv_min, sizeof(float3), cudaMemcpyHostToDevice);

	h_CurUploadMeshCount++;
	h_CurUploadVertexCount += vertexCount;
	h_CurUploadIndexCount += indexCount;

	return h_CurUploadMeshCount - 1;
}


__constant__ d_Color32 BGColor;
__constant__ int2 ScreenSize;
__constant__ int ScreenBufferSize;

__constant__ float4 WorldToViewMat[4];
__constant__ float4 WorldToNDCMat[4];
__constant__ float CamNear;
__constant__ float Inv_CamDis;

d_Color32* d_ColorBuffer;
float* d_DepthBuffer;
int h_ScreenSize;

Matrix4x4 h_WorldToViewMat;
Matrix4x4 h_WorldToNDCMat;

void CU_ChangeScreenSize(const ScreenPoint& screenSize)
{
	h_ScreenSize = screenSize.X * screenSize.Y;
	cudaMemcpyToSymbol(ScreenSize, &screenSize, sizeof(int2));
	cudaMemcpyToSymbol(ScreenBufferSize, &h_ScreenSize, sizeof(int));
}

void CU_BufferInit(Color32*& colorBuffer, float*& depthBuffer)
{
	cudaMalloc(&d_ColorBuffer, sizeof(d_Color32) * h_ScreenSize);
	cudaMalloc(&d_DepthBuffer, sizeof(float) * h_ScreenSize);
	cudaMallocHost(&depthBuffer, sizeof(float) * h_ScreenSize);
	cudaHostRegister(colorBuffer, sizeof(d_Color32) * h_ScreenSize, cudaHostRegisterMapped);
}
void CU_BufferFree(Color32*& colorBuffer, float*& depthBuffer)
{
	cudaFree(d_ColorBuffer);
	cudaFree(d_DepthBuffer);
	cudaFreeHost(depthBuffer);
	cudaHostUnregister(colorBuffer);
	cudaDeviceSynchronize();
}
__global__ void BG_Depth_Init(d_Color32* colorBuffer, float* depthBuffer)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= ScreenBufferSize)
	{
		return;
	}
	colorBuffer[idx] = BGColor;
	depthBuffer[idx] = 1.f;
}
void CU_BG_Depth_CamInfoSetting(const Color32& bgColor, const ScreenPoint& screenSize, Camera* pMainCam)
{
	cudaMemcpyToSymbolAsync(BGColor, &bgColor, sizeof(int), 0, cudaMemcpyHostToDevice, S_BGDepthInit);
	dim3 block;
	int thread;
	Block_Thread_Size(h_ScreenSize, block, thread);
	BG_Depth_Init << <block, thread, 0, S_BGDepthInit >> > (d_ColorBuffer, d_DepthBuffer);

	h_WorldToNDCMat = pMainCam->GetWorldToNDCMat();
	cudaMemcpyToSymbolAsync(WorldToNDCMat, &h_WorldToNDCMat, sizeof(float4) * 4, 0, cudaMemcpyHostToDevice);
	h_WorldToViewMat = pMainCam->GetWorldToViewMat();
	cudaMemcpyToSymbolAsync(WorldToViewMat, &h_WorldToViewMat, sizeof(float4) * 4, 0, cudaMemcpyHostToDevice);

	float camNear, camFar, inv_CamDis;
	pMainCam->GetNearFar(camNear, camFar);
	cudaMemcpyToSymbolAsync(CamNear, &camNear, sizeof(float), 0, cudaMemcpyHostToDevice);
	inv_CamDis = 1 / (camFar - camNear);
	cudaMemcpyToSymbolAsync(Inv_CamDis, &inv_CamDis, sizeof(float), 0, cudaMemcpyHostToDevice);

}

void CU_SetRenderState()
{
	//마테리얼 추가 후 수정될 예정
}

#pragma region DrawCall 처리과정

__constant__ int MaterialIdx;
__constant__ int GoCount;
int* d_MeshIdx;
int* h_MeshIdx;
d_Matrix4x4* d_LocalToNDCMat;
int h_GoCount;

#pragma region BoundCheckBox
//boundBox 일단 박스로 통일
int* d_BCR;
float* d_log;
__global__ void BoundCheckBox(int* d_MeshIdx, int* d_BCR, d_Matrix4x4* d_LocalToNDCMat, float3* d_BvBox_Min, float3* d_BvBox_Max)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= GoCount)
	{
		return;
	}

	d_Matrix4x4 LocalToNDCMat = d_LocalToNDCMat[idx];
	float4 d_Frustum[6];
	float f12 = -LocalToNDCMat.Ele[12];
	float f13 = -LocalToNDCMat.Ele[13];
	float f14 = -LocalToNDCMat.Ele[14];
	float f15 = -LocalToNDCMat.Ele[15];

	//+y
	d_Frustum[0].x = f12 + LocalToNDCMat.Ele[4];
	d_Frustum[0].y = f13 + LocalToNDCMat.Ele[5];
	d_Frustum[0].z = f14 + LocalToNDCMat.Ele[6];
	d_Frustum[0].w = f15 + LocalToNDCMat.Ele[7];
	//-y
	d_Frustum[1].x = f12 - LocalToNDCMat.Ele[4];
	d_Frustum[1].y = f13 - LocalToNDCMat.Ele[5];
	d_Frustum[1].z = f14 - LocalToNDCMat.Ele[6];
	d_Frustum[1].w = f15 - LocalToNDCMat.Ele[7];
	//+x
	d_Frustum[2].x = f12 + LocalToNDCMat.Ele[0];
	d_Frustum[2].y = f13 + LocalToNDCMat.Ele[1];
	d_Frustum[2].z = f14 + LocalToNDCMat.Ele[2];
	d_Frustum[2].w = f15 + LocalToNDCMat.Ele[3];
	//-x
	d_Frustum[3].x = f12 - LocalToNDCMat.Ele[0];
	d_Frustum[3].y = f13 - LocalToNDCMat.Ele[1];
	d_Frustum[3].z = f14 - LocalToNDCMat.Ele[2];
	d_Frustum[3].w = f15 - LocalToNDCMat.Ele[3];
	//+z
	d_Frustum[4].x = f12 + LocalToNDCMat.Ele[8];
	d_Frustum[4].y = f13 + LocalToNDCMat.Ele[9];
	d_Frustum[4].z = f14 + LocalToNDCMat.Ele[10];
	d_Frustum[4].w = f15 + LocalToNDCMat.Ele[11];
	//-z
	d_Frustum[5].x = f12 - LocalToNDCMat.Ele[8];
	d_Frustum[5].y = f13 - LocalToNDCMat.Ele[9];
	d_Frustum[5].z = f14 - LocalToNDCMat.Ele[10];
	d_Frustum[5].w = f15 - LocalToNDCMat.Ele[11];

	float total_pow, inv_sqrt;

	total_pow = d_Frustum[0].x * d_Frustum[0].x + d_Frustum[0].y * d_Frustum[0].y + d_Frustum[0].z * d_Frustum[0].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[0].x *= inv_sqrt;
	d_Frustum[0].y *= inv_sqrt;
	d_Frustum[0].z *= inv_sqrt;
	d_Frustum[0].w *= inv_sqrt;

	total_pow = d_Frustum[1].x * d_Frustum[1].x + d_Frustum[1].y * d_Frustum[1].y + d_Frustum[1].z * d_Frustum[1].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[1].x *= inv_sqrt;
	d_Frustum[1].y *= inv_sqrt;
	d_Frustum[1].z *= inv_sqrt;
	d_Frustum[1].w *= inv_sqrt;

	total_pow = d_Frustum[2].x * d_Frustum[2].x + d_Frustum[2].y * d_Frustum[2].y + d_Frustum[2].z * d_Frustum[2].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[2].x *= inv_sqrt;
	d_Frustum[2].y *= inv_sqrt;
	d_Frustum[2].z *= inv_sqrt;
	d_Frustum[2].w *= inv_sqrt;

	total_pow = d_Frustum[3].x * d_Frustum[3].x + d_Frustum[3].y * d_Frustum[3].y + d_Frustum[3].z * d_Frustum[3].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[3].x *= inv_sqrt;
	d_Frustum[3].y *= inv_sqrt;
	d_Frustum[3].z *= inv_sqrt;
	d_Frustum[3].w *= inv_sqrt;

	total_pow = d_Frustum[4].x * d_Frustum[4].x + d_Frustum[4].y * d_Frustum[4].y + d_Frustum[4].z * d_Frustum[4].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[4].x *= inv_sqrt;
	d_Frustum[4].y *= inv_sqrt;
	d_Frustum[4].z *= inv_sqrt;
	d_Frustum[4].w *= inv_sqrt;

	total_pow = d_Frustum[5].x * d_Frustum[5].x + d_Frustum[5].y * d_Frustum[5].y + d_Frustum[5].z * d_Frustum[5].z;
	inv_sqrt = 1 / sqrtf(total_pow);
	d_Frustum[5].x *= inv_sqrt;
	d_Frustum[5].y *= inv_sqrt;
	d_Frustum[5].z *= inv_sqrt;
	d_Frustum[5].w *= inv_sqrt;

	int xDir, yDir, zDir, wDir; // 0 = 양수 or 0 , 1 = 음수
	float3 bv_Min = d_BvBox_Min[d_MeshIdx[idx]];
	float3 bv_Max = d_BvBox_Max[d_MeshIdx[idx]];

	float3 innerPos, outPos;
	float inDis, outDis;

	bool isIntersect = false;

	//inDis > 0 = outSide, outDis < 0 inSide
	//Inside = 1,
	//Intersect = 2,
	//Outside = 4

	float dirMulMin, dirMulMax;
	xDir = signbit(d_Frustum[0].x);
	yDir = signbit(d_Frustum[0].y);
	zDir = signbit(d_Frustum[0].z);
	wDir = signbit(d_Frustum[0].w);
	/*innerPos.x = bv_Min.x * (1 - xDir) + bv_Max.x * xDir;
	outPos.x = bv_Min.x * xDir + bv_Max.x * (1 - xDir);
	innerPos.y = bv_Min.y * (1 - yDir) + bv_Max.y * yDir;
	outPos.y = bv_Min.y * yDir + bv_Max.y * (1 - yDir);
	innerPos.z = bv_Min.z * (1 - zDir) + bv_Max.x * zDir;
	outPos.z = bv_Min.z * zDir + bv_Max.x * (1 - zDir);*/ //해당 식 중복곱 캐싱한것 1000배 했을때 0.5ms정도 빨라지는듯
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[0].x * innerPos.x + d_Frustum[0].y * innerPos.y + d_Frustum[0].z * innerPos.z + d_Frustum[0].w;
	outDis = d_Frustum[0].x * outPos.x + d_Frustum[0].y * outPos.y + d_Frustum[0].z * outPos.z + d_Frustum[0].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	xDir = signbit(d_Frustum[1].x);
	yDir = signbit(d_Frustum[1].y);
	zDir = signbit(d_Frustum[1].z);
	wDir = signbit(d_Frustum[1].w);
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[1].x * innerPos.x + d_Frustum[1].y * innerPos.y + d_Frustum[1].z * innerPos.z + d_Frustum[1].w;
	outDis = d_Frustum[1].x * outPos.x + d_Frustum[1].y * outPos.y + d_Frustum[1].z * outPos.z + d_Frustum[1].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	xDir = signbit(d_Frustum[2].x);
	yDir = signbit(d_Frustum[2].y);
	zDir = signbit(d_Frustum[2].z);
	wDir = signbit(d_Frustum[2].w);
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[2].x * innerPos.x + d_Frustum[2].y * innerPos.y + d_Frustum[2].z * innerPos.z + d_Frustum[2].w;
	outDis = d_Frustum[2].x * outPos.x + d_Frustum[2].y * outPos.y + d_Frustum[2].z * outPos.z + d_Frustum[2].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	xDir = signbit(d_Frustum[3].x);
	yDir = signbit(d_Frustum[3].y);
	zDir = signbit(d_Frustum[3].z);
	wDir = signbit(d_Frustum[3].w);
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[3].x * innerPos.x + d_Frustum[3].y * innerPos.y + d_Frustum[3].z * innerPos.z + d_Frustum[3].w;
	outDis = d_Frustum[3].x * outPos.x + d_Frustum[3].y * outPos.y + d_Frustum[3].z * outPos.z + d_Frustum[3].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	xDir = signbit(d_Frustum[4].x);
	yDir = signbit(d_Frustum[4].y);
	zDir = signbit(d_Frustum[4].z);
	wDir = signbit(d_Frustum[4].w);
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[4].x * innerPos.x + d_Frustum[4].y * innerPos.y + d_Frustum[4].z * innerPos.z + d_Frustum[4].w;
	outDis = d_Frustum[4].x * outPos.x + d_Frustum[4].y * outPos.y + d_Frustum[4].z * outPos.z + d_Frustum[4].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	xDir = signbit(d_Frustum[5].x);
	yDir = signbit(d_Frustum[5].y);
	zDir = signbit(d_Frustum[5].z);
	wDir = signbit(d_Frustum[5].w);
	dirMulMin = bv_Min.x * xDir;
	dirMulMax = bv_Max.x * xDir;
	innerPos.x = bv_Min.x - dirMulMin + dirMulMax;
	outPos.x = dirMulMin + bv_Max.x - dirMulMax;
	dirMulMin = bv_Min.y * yDir;
	dirMulMax = bv_Max.y * yDir;
	innerPos.y = bv_Min.y - dirMulMin + dirMulMax;
	outPos.y = dirMulMin + bv_Max.y - dirMulMax;
	dirMulMin = bv_Min.z * zDir;
	dirMulMax = bv_Max.z * zDir;
	innerPos.z = bv_Min.z - dirMulMin + dirMulMax;
	outPos.z = dirMulMin + bv_Max.z - dirMulMax;
	inDis = d_Frustum[5].x * innerPos.x + d_Frustum[5].y * innerPos.y + d_Frustum[5].z * innerPos.z + d_Frustum[5].w;
	outDis = d_Frustum[5].x * outPos.x + d_Frustum[5].y * outPos.y + d_Frustum[5].z * outPos.z + d_Frustum[5].w;
	if (inDis > 0)
	{
		d_BCR[idx] = 4;
		return;
	}
	else if (outDis >= 0)
	{
		isIntersect = true;
	}

	if (isIntersect)
	{
		d_BCR[idx] = 2;
	}
	else
	{
		d_BCR[idx] = 1;
	}
}


int h_OutsideCount = 0;
int h_InsideCount = 0;
int h_IntersectCount = 0;
int h_DrawGoCount = 0;

int* h_BCR;
void CU_BoundCheckBox()
{
	dim3 block;
	int thread;
	Block_Thread_Size(h_GoCount, block, thread);
	BoundCheckBox << <block, thread >> > (d_MeshIdx, d_BCR, d_LocalToNDCMat, d_BvBox_Min, d_BvBox_Max);
	cudaMemcpy(h_BCR, d_BCR, sizeof(int) * h_GoCount, cudaMemcpyDeviceToHost);
}

#pragma endregion
#pragma region VertexSettingAndClipping
#pragma region functions
__device__ bool CheckisOut(int* pdirIdx, float4* pVt4, float* pSmallNumber)
{
	switch (*pdirIdx)
	{
	case 0:
		return pVt4->w < *pSmallNumber;
	case 1:
		return pVt4->z < -*pSmallNumber;
	case 2:
		return pVt4->z > 1 + *pSmallNumber;
	case 3:
		return pVt4->x > pVt4->w + *pSmallNumber;
	case 4:
		return pVt4->x < -pVt4->w - *pSmallNumber;
	case 5:
		return pVt4->y > pVt4->w + *pSmallNumber;
	case 6:
		return pVt4->y < -pVt4->w - *pSmallNumber;
	}
}
__device__ void FrustumClipping(int* pdirIdx, float4* pOutPos, float4* pInPos, float2* pOutUV, float2* pInUV, float4* pDesPos, float2* pDesUV)
{
	float t, s;
	switch (*pdirIdx)
	{
	case 0:
		t = pInPos->w / (pInPos->w - pOutPos->w);
		s = 1 - t;
		pDesPos->x = pOutPos->x * t + pInPos->x * s;
		pDesPos->y = pOutPos->y * t + pInPos->y * s;
		pDesPos->z = pOutPos->z * t + pInPos->z * s;
		pDesPos->w = 0;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 1:
		t = pInPos->z / (pInPos->z - pOutPos->z);
		s = 1 - t;
		pDesPos->x = pOutPos->x * t + pInPos->x * s;
		pDesPos->y = pOutPos->y * t + pInPos->y * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->z = 0;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 2:
		t = (pInPos->z - 1) / (pInPos->z - pOutPos->z);
		s = 1 - t;
		pDesPos->x = pOutPos->x * t + pInPos->x * s;
		pDesPos->y = pOutPos->y * t + pInPos->y * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->z = 1;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 3:
		t = (pInPos->w - pInPos->x) / (pInPos->w - pInPos->x - pOutPos->w + pOutPos->x);
		s = 1 - t;
		pDesPos->y = pOutPos->y * t + pInPos->y * s;
		pDesPos->z = pOutPos->z * t + pInPos->z * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->x = pDesPos->w;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 4:
		t = (pInPos->w + pInPos->x) / (pInPos->w + pInPos->x - pOutPos->w - pOutPos->x);
		s = 1 - t;
		pDesPos->y = pOutPos->y * t + pInPos->y * s;
		pDesPos->z = pOutPos->z * t + pInPos->z * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->x = -pDesPos->w;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 5:
		t = (pInPos->w - pInPos->y) / (pInPos->w - pInPos->y - pOutPos->w + pOutPos->y);
		s = 1 - t;
		pDesPos->x = pOutPos->x * t + pInPos->x * s;
		pDesPos->z = pOutPos->z * t + pInPos->z * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->y = pDesPos->w;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	case 6:
		t = (pInPos->w + pInPos->y) / (pInPos->w + pInPos->y - pOutPos->w - pOutPos->y);
		s = 1 - t;
		pDesPos->x = pOutPos->x * t + pInPos->x * s;
		pDesPos->z = pOutPos->z * t + pInPos->z * s;
		pDesPos->w = pOutPos->w * t + pInPos->w * s;
		pDesPos->y = -pDesPos->w;

		pDesUV->x = pOutUV->x * t + pInUV->x * s;
		pDesUV->y = pOutUV->y * t + pInUV->y * s;
		break;
	}
}
#pragma endregion
float4* d_VertexPos;
float2* d_VertexUV;
bool* d_isDraw;
int* d_DrawTriangleIdxPrefixSum;
bool* h_isIntersect;
bool* d_isIntersect;
int* h_TriangleIdx;
int* d_TriangleIdx;
int* h_TriangleGoIdx;
int* d_TriangleGoIdx;
int* h_VertexStartIdx;
int* d_VertexStartIdx;

int h_TriangleCount;
__constant__ int TriangleCount;
int h_VertexCountAfterClip;
__constant__ int VertexCountAfterClip;
int h_TriangleCountAfterClip;
__constant__ int TriangleCountAfterClip;


__global__ void VertexSetting(float4* d_VertexPos, float2* d_VertexUV, bool* d_isDraw, bool* d_isIntersect, int* d_TriangleIdx, int* d_TriangleGoIdx, int* d_VertexStartIdx, d_Matrix4x4* d_LocalToNDCMat,
	int* d_MeshIdx, int* d_MeshIndexStartIdx, int* d_MeshIndices, int* d_MeshVertexStartIdx, float3* d_MeshVertices, float2* d_MeshUVs)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= TriangleCount)
	{
		return;
	}

	//return; // 6~7

	int triangleIdx = d_TriangleIdx[idx];
	int goIdx = d_TriangleGoIdx[idx];
	int meshIdx = d_MeshIdx[goIdx]; //메쉬의 번호

	int meshIndex; //메쉬 안에서의 인덱스
	int vStartIdx = d_VertexStartIdx[idx];
	int vIdx;
	float3 meshVertex;
	float inv_w;
	float4 VertexPos;
	d_Matrix4x4 localToNDCMat = d_LocalToNDCMat[goIdx];
	int curVertCount = 3;
	int curTriangleCount = curVertCount - 2;

	vIdx = vStartIdx + 0;
	meshIndex = d_MeshIndices[d_MeshIndexStartIdx[meshIdx] + triangleIdx * 3 + 0];
	meshVertex = d_MeshVertices[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	d_VertexUV[vIdx] = d_MeshUVs[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	VertexPos.x = localToNDCMat.Ele[0] * meshVertex.x + localToNDCMat.Ele[1] * meshVertex.y + localToNDCMat.Ele[2] * meshVertex.z + localToNDCMat.Ele[3];
	VertexPos.y = localToNDCMat.Ele[4] * meshVertex.x + localToNDCMat.Ele[5] * meshVertex.y + localToNDCMat.Ele[6] * meshVertex.z + localToNDCMat.Ele[7];
	VertexPos.w = localToNDCMat.Ele[12] * meshVertex.x + localToNDCMat.Ele[13] * meshVertex.y + localToNDCMat.Ele[14] * meshVertex.z + localToNDCMat.Ele[15];
	VertexPos.z = ((VertexPos.w - CamNear) * Inv_CamDis);
	d_VertexPos[vIdx] = VertexPos;

	vIdx = vStartIdx + 1;
	meshIndex = d_MeshIndices[d_MeshIndexStartIdx[meshIdx] + triangleIdx * 3 + 1];
	meshVertex = d_MeshVertices[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	d_VertexUV[vIdx] = d_MeshUVs[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	VertexPos.x = localToNDCMat.Ele[0] * meshVertex.x + localToNDCMat.Ele[1] * meshVertex.y + localToNDCMat.Ele[2] * meshVertex.z + localToNDCMat.Ele[3];
	VertexPos.y = localToNDCMat.Ele[4] * meshVertex.x + localToNDCMat.Ele[5] * meshVertex.y + localToNDCMat.Ele[6] * meshVertex.z + localToNDCMat.Ele[7];
	VertexPos.w = localToNDCMat.Ele[12] * meshVertex.x + localToNDCMat.Ele[13] * meshVertex.y + localToNDCMat.Ele[14] * meshVertex.z + localToNDCMat.Ele[15];
	VertexPos.z = ((VertexPos.w - CamNear) * Inv_CamDis);
	d_VertexPos[vIdx] = VertexPos;

	vIdx = vStartIdx + 2;
	meshIndex = d_MeshIndices[d_MeshIndexStartIdx[meshIdx] + triangleIdx * 3 + 2];
	meshVertex = d_MeshVertices[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	d_VertexUV[vIdx] = d_MeshUVs[d_MeshVertexStartIdx[meshIdx] + meshIndex];
	VertexPos.x = localToNDCMat.Ele[0] * meshVertex.x + localToNDCMat.Ele[1] * meshVertex.y + localToNDCMat.Ele[2] * meshVertex.z + localToNDCMat.Ele[3];
	VertexPos.y = localToNDCMat.Ele[4] * meshVertex.x + localToNDCMat.Ele[5] * meshVertex.y + localToNDCMat.Ele[6] * meshVertex.z + localToNDCMat.Ele[7];
	VertexPos.w = localToNDCMat.Ele[12] * meshVertex.x + localToNDCMat.Ele[13] * meshVertex.y + localToNDCMat.Ele[14] * meshVertex.z + localToNDCMat.Ele[15];
	VertexPos.z = ((VertexPos.w - CamNear) * Inv_CamDis);
	d_VertexPos[vIdx] = VertexPos;

	//return; // 8~9

	if (d_isIntersect[idx])
	{
		//clipping 처리
		int dirIdx;
		int prev, cur, next;
		bool arr_isOut[9];
		float4 arr_Pos_ClipBefore[9];
		float4 arr_Pos_ClipResult[9];
		float2 arr_UV_ClipBefore[9];
		float2 arr_UV_ClipResult[9];
		float small_Number = 1.e-6f;
		int writeIdx = 0;

		arr_Pos_ClipBefore[0] = d_VertexPos[vStartIdx];
		arr_Pos_ClipBefore[1] = d_VertexPos[vStartIdx + 1];
		arr_Pos_ClipBefore[2] = d_VertexPos[vStartIdx + 2];
		arr_UV_ClipBefore[0] = d_VertexUV[vStartIdx];
		arr_UV_ClipBefore[1] = d_VertexUV[vStartIdx + 1];
		arr_UV_ClipBefore[2] = d_VertexUV[vStartIdx + 2];
		//return; // 9~10
#pragma region forloop
		for (int dirIdx = 0; dirIdx < 7; dirIdx++)
		{
			for (int i = 0; i < curVertCount; i++)
			{
				arr_isOut[i] = CheckisOut(&dirIdx, &arr_Pos_ClipBefore[i], &small_Number);
			}

			for (int i = 0; i < curVertCount; i++)
			{
				//통일되는 분기라 상관없음
				if (i == 0)
				{
					prev = curVertCount - 2;
					cur = curVertCount - 1;
					next = 0;
				}
				else if (i == 1)
				{
					prev = curVertCount - 1;
					cur = 0;
					next = 1;
				}
				else
				{
					prev = i - 2;
					cur = i - 1;
					next = i;
				}
				if (arr_isOut[cur] == true)
				{
					if (arr_isOut[prev] == false)
					{
						FrustumClipping(&dirIdx, &arr_Pos_ClipBefore[cur], &arr_Pos_ClipBefore[prev], &arr_UV_ClipBefore[cur], &arr_UV_ClipBefore[prev], &arr_Pos_ClipResult[writeIdx], &arr_UV_ClipResult[writeIdx]);
						writeIdx++;
					}
					if (arr_isOut[next] == false)
					{
						FrustumClipping(&dirIdx, &arr_Pos_ClipBefore[cur], &arr_Pos_ClipBefore[next], &arr_UV_ClipBefore[cur], &arr_UV_ClipBefore[next], &arr_Pos_ClipResult[writeIdx], &arr_UV_ClipResult[writeIdx]);
						writeIdx++;
					}
				}
				else
				{
					//그대로 대입
					arr_Pos_ClipResult[writeIdx] = arr_Pos_ClipBefore[cur];
					arr_UV_ClipResult[writeIdx] = arr_UV_ClipBefore[cur];
					writeIdx++;
				}
			}
			curVertCount = writeIdx;
			writeIdx = 0;
			for (int i = 0; i < curVertCount; i++)
			{
				arr_isOut[i] = false;
				arr_Pos_ClipBefore[i] = arr_Pos_ClipResult[i];
				arr_UV_ClipBefore[i] = arr_UV_ClipResult[i];
			}
		}

#pragma endregion

		//이전 forloop의 마지막 초기화는 생략
		//curVertCount = 현재 클리핑 후 점 갯수
		//arr_Pos_ClipResult에 결과 들어있음
		curTriangleCount = curVertCount - 2;
		for (int i = 0; i < curTriangleCount; i++)
		{
			vIdx = vStartIdx + i * 3;
			d_VertexPos[vIdx] = arr_Pos_ClipResult[0];
			d_VertexPos[vIdx + 1] = arr_Pos_ClipResult[i + 1];
			d_VertexPos[vIdx + 2] = arr_Pos_ClipResult[i + 2];
			d_VertexUV[vIdx] = arr_UV_ClipResult[0];
			d_VertexUV[vIdx + 1] = arr_UV_ClipResult[i + 1];
			d_VertexUV[vIdx + 2] = arr_UV_ClipResult[i + 2];
		}
	}
	int value = vStartIdx / 3;
	for (int i = 0; i < curTriangleCount; i++)
	{
		d_isDraw[value + i] = true;

		vIdx = vStartIdx + i * 3;
		inv_w = 1 / d_VertexPos[vIdx].w;
		d_VertexPos[vIdx].x *= inv_w;
		d_VertexPos[vIdx].y *= inv_w;
		inv_w = 1 / d_VertexPos[vIdx + 1].w;
		d_VertexPos[vIdx + 1].x *= inv_w;
		d_VertexPos[vIdx + 1].y *= inv_w;
		inv_w = 1 / d_VertexPos[vIdx + 2].w;
		d_VertexPos[vIdx + 2].x *= inv_w;
		d_VertexPos[vIdx + 2].y *= inv_w;
	}
}

void CU_VertexArraySetting()
{
	cudaStream_t S_VertexArraySetting;
	cudaStreamCreate(&S_VertexArraySetting);
	int insideCount = 0;
	int intersectCount = 0;
	int outsideCount = 0;
	int insideTriangleCount = 0;
	int intersectTriangleCount = 0;
	for (int i = 0; i < h_GoCount; i++)
	{
		switch (h_BCR[i])
		{
		case  1: //inside
			insideTriangleCount += h_MeshTriangleCountLookUpTable[h_MeshIdx[i]];
			insideCount++;
			break;
		case 2: //intersect
			intersectTriangleCount += h_MeshTriangleCountLookUpTable[h_MeshIdx[i]];
			intersectCount++;
			break;
		case 4: //outside
			break;
		}
	}

	h_TriangleCount = insideTriangleCount + intersectTriangleCount;
	h_VertexCountAfterClip = insideTriangleCount * 3 + intersectTriangleCount * 21;
	h_TriangleCountAfterClip = h_VertexCountAfterClip / 3;

	//intersectTriangleCount 는 한점clipping 이 최대 6번까지 발생가능 그럼 삼각형이 7개이므로 점은 21개
	static int TriangleArrayCount = 0;
	if (TriangleArrayCount < h_TriangleCount)
	{
		if (TriangleArrayCount != 0)
		{
			delete[] h_isIntersect;
			cudaFree(d_isIntersect);
			delete[] h_TriangleIdx;
			cudaFree(d_TriangleIdx);
			delete[] h_TriangleGoIdx;
			cudaFree(d_TriangleGoIdx);
			delete[] h_VertexStartIdx;
			cudaFree(d_VertexStartIdx);
		}
		TriangleArrayCount = h_TriangleCount + (h_TriangleCount >> 2);
		h_isIntersect = new bool[TriangleArrayCount];
		cudaMalloc(&d_isIntersect, sizeof(bool) * TriangleArrayCount);
		h_TriangleIdx = new int[TriangleArrayCount];
		cudaMalloc(&d_TriangleIdx, sizeof(int) * TriangleArrayCount);
		h_TriangleGoIdx = new int[TriangleArrayCount];
		cudaMalloc(&d_TriangleGoIdx, sizeof(int) * TriangleArrayCount);
		h_VertexStartIdx = new int[TriangleArrayCount];
		cudaMalloc(&d_VertexStartIdx, sizeof(int) * TriangleArrayCount);
	}

	static int VertexArrayCountAfterClip = 0;
	static int TriangleArrayCountAfterClip = 0;
	if (VertexArrayCountAfterClip < h_VertexCountAfterClip)
	{
		if (VertexArrayCountAfterClip != 0)
		{
			cudaFree(d_VertexPos);
			cudaFree(d_VertexUV);
			cudaFree(d_isDraw);
			cudaFree(d_DrawTriangleIdxPrefixSum);
		}
		VertexArrayCountAfterClip = h_VertexCountAfterClip + (h_VertexCountAfterClip >> 2);
		TriangleArrayCountAfterClip = VertexArrayCountAfterClip / 3;
		cudaMalloc(&d_isDraw, sizeof(bool) * TriangleArrayCountAfterClip);
		cudaMalloc(&d_DrawTriangleIdxPrefixSum, sizeof(int) * TriangleArrayCountAfterClip);
		cudaMalloc(&d_VertexPos, sizeof(float4) * VertexArrayCountAfterClip);
		cudaMalloc(&d_VertexUV, sizeof(float4) * VertexArrayCountAfterClip);
	}

	cudaMemcpyToSymbolAsync(TriangleCount, &h_TriangleCount, sizeof(int), 0, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemcpyToSymbolAsync(VertexCountAfterClip, &h_VertexCountAfterClip, sizeof(int), 0, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemcpyToSymbolAsync(TriangleCountAfterClip, &h_TriangleCountAfterClip, sizeof(int), 0, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemsetAsync(d_isDraw, 0, sizeof(bool) * h_VertexCountAfterClip / 3, S_VertexArraySetting);

	int curMeshTriangleCount = 0;
	int curTriangleIdx = 0;
	int curVertexIdx = 0;

	for (int i = 0; i < h_GoCount; i++)
	{
		switch (h_BCR[i])
		{
		case 1: //inside
			curMeshTriangleCount = h_MeshTriangleCountLookUpTable[h_MeshIdx[i]];
			for (int j = 0; j < curMeshTriangleCount; j++)
			{
				h_TriangleIdx[curTriangleIdx] = j;
				h_TriangleGoIdx[curTriangleIdx] = i;
				h_isIntersect[curTriangleIdx] = false;
				h_VertexStartIdx[curTriangleIdx] = curVertexIdx;
				curTriangleIdx++;
				curVertexIdx += 3;
			}
			break;
		case 2: //intersect
			curMeshTriangleCount = h_MeshTriangleCountLookUpTable[h_MeshIdx[i]];
			for (int j = 0; j < curMeshTriangleCount; j++)
			{
				h_TriangleIdx[curTriangleIdx] = j;
				h_TriangleGoIdx[curTriangleIdx] = i;
				h_isIntersect[curTriangleIdx] = true;
				h_VertexStartIdx[curTriangleIdx] = curVertexIdx;
				curTriangleIdx++;
				curVertexIdx += 21;
			}
			break;
		}
	}
	cudaMemcpyAsync(d_TriangleIdx, h_TriangleIdx, sizeof(int) * h_TriangleCount, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemcpyAsync(d_TriangleGoIdx, h_TriangleGoIdx, sizeof(int) * h_TriangleCount, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemcpyAsync(d_isIntersect, h_isIntersect, sizeof(bool) * h_TriangleCount, cudaMemcpyHostToDevice, S_VertexArraySetting);
	cudaMemcpyAsync(d_VertexStartIdx, h_VertexStartIdx, sizeof(int) * h_TriangleCount, cudaMemcpyHostToDevice, S_VertexArraySetting);

	dim3 block;
	int thread;
	Block_Thread_Size(h_TriangleCount, block, thread);

	VertexSetting << <block, thread, 0, S_VertexArraySetting >> > (d_VertexPos, d_VertexUV, d_isDraw, d_isIntersect, d_TriangleIdx, d_TriangleGoIdx, d_VertexStartIdx, d_LocalToNDCMat,
		d_MeshIdx, d_MeshIndexStartIdx, d_MeshIndices, d_MeshVertexStartIdx, d_MeshVertices, d_MeshUVs);
	cudaStreamSynchronize(S_VertexArraySetting);
	cudaStreamDestroy(S_VertexArraySetting);
}
#pragma endregion

__global__ void Culling(float4* d_VertexPos, bool* d_isDraw, int* d_DrawTriangleIdxPrefixSum)
{
	extern __shared__ int shared[];
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= TriangleCountAfterClip)
	{
		return;
	}

	if (d_isDraw[idx] == true)
	{
		float4 pos0 = d_VertexPos[idx * 3];
		float4 pos1 = d_VertexPos[idx * 3 + 1];
		float4 pos2 = d_VertexPos[idx * 3 + 2];

		float x_12, y_12, x_13, y_13;
		x_12 = pos1.x - pos0.x;
		y_12 = pos1.y - pos0.y;
		x_13 = pos2.x - pos0.x;
		y_13 = pos2.y - pos0.y;
		d_isDraw[idx] = x_12 * y_13 - y_12 * x_13 < 0;
	}

	shared[threadIdx.x] = d_isDraw[idx];
	for (int stride = 1; stride < blockDim.x; stride <<= 1)
	{
		__syncthreads();
		if (threadIdx.x >= stride)
		{
			shared[threadIdx.x] += shared[threadIdx.x - stride];
		}
	}
	d_DrawTriangleIdxPrefixSum[idx] = shared[threadIdx.x];
}
__global__ void SetDrawTriangleIdxPrefixCom(bool* d_isDraw, int* d_DrawTriangleIdxPrefixSum, int* d_DrawTriangleIdxPrefixSumCom)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= TriangleCountAfterClip)
	{
		return;
	}
	if (d_isDraw[idx] == true)
	{
		int comIdx = d_DrawTriangleIdxPrefixSum[idx] - 1;
		d_DrawTriangleIdxPrefixSumCom[comIdx] = idx;
	}
}

int* d_DrawTriangleIdxPrefixSumCom;
float4* d_DrawVertexPos;
float2* d_DrawVertexUV;

__constant__ int DrawTriangleCount;
int h_DrawTriangleCount;
int* d_VerticalBottom;
int* d_VerticalHeightPrefixSum;

int h_DrawTriangleHeightCount;
__constant__ int DrawTriangleHeightCount;
int* d_DrawTriangleHeightTriangleIdx;
int* d_DrawTriangleHeight;
int* d_DrawTriangleHeightLeft;
int* d_DrawTriangleHeightRight;
float2* d_DTHLeftUV;
float2* d_DTHRightUV;
float* d_DTHLeftDepth;
float* d_DTHRightDepth;
int* d_FragmentCountPrefixSum;

int h_FragmentCount;
__constant__ int FragmentCount;
int* d_FragmentDTHIdx; //DTH = DrawTriangleHeight


__global__ void DrawVertexSetting_GetVerticalRange(float4* d_VertexPos, float2* d_VertexUV, float4* d_DrawVertexPos, float2* d_DrawVertexUV, int* d_DrawTriangleIdxPrefixSumCom, int* d_VerticalBottom, int* d_VerticalHeightPrefixSum)
{
	extern __shared__ int shared[];
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= DrawTriangleCount) return;

	int vIdx0 = d_DrawTriangleIdxPrefixSumCom[idx] * 3;
	int vIdx1 = vIdx0 + 1;
	int vIdx2 = vIdx0 + 2;
	int DrawvIdx0 = idx * 3;
	int DrawvIdx1 = DrawvIdx0 + 1;
	int DrawvIdx2 = DrawvIdx0 + 2;

	float4 vPos0 = d_VertexPos[vIdx0];
	float4 vPos1 = d_VertexPos[vIdx1];
	float4 vPos2 = d_VertexPos[vIdx2];

	float halfScreenX = ScreenSize.x * 0.5f;
	float halfScreenY = ScreenSize.y * 0.5f;

	vPos0.x = vPos0.x * halfScreenX + halfScreenX;
	vPos1.x = vPos1.x * halfScreenX + halfScreenX;
	vPos2.x = vPos2.x * halfScreenX + halfScreenX;
	vPos0.y = -(vPos0.y * halfScreenY) + halfScreenY;
	vPos1.y = -(vPos1.y * halfScreenY) + halfScreenY;
	vPos2.y = -(vPos2.y * halfScreenY) + halfScreenY;

	d_DrawVertexUV[DrawvIdx0] = d_VertexUV[vIdx0];
	d_DrawVertexUV[DrawvIdx1] = d_VertexUV[vIdx1];
	d_DrawVertexUV[DrawvIdx2] = d_VertexUV[vIdx2];
	d_DrawVertexPos[DrawvIdx0] = vPos0;
	d_DrawVertexPos[DrawvIdx1] = vPos1;
	d_DrawVertexPos[DrawvIdx2] = vPos2;

	float top = max(max(vPos0.y, vPos1.y), vPos2.y);
	float bottom = min(min(vPos0.y, vPos1.y), vPos2.y);

	int iTop, iBottom;
	int iValue = (int)bottom;
	int addValue = bottom > iValue + 0.5f ? 1 : 0;
	iBottom = iValue + addValue;

	iValue = (int)top;
	addValue = top > iValue + 0.5f ? 0 : -1;
	iTop = iValue + addValue;

	d_VerticalBottom[idx] = iBottom;
	shared[threadIdx.x] = iTop - iBottom + 1;
	for (int stride = 1; stride < blockDim.x; stride <<= 1)
	{
		__syncthreads();
		if (threadIdx.x >= stride)
		{
			shared[threadIdx.x] += shared[threadIdx.x - stride];
		}
	}

	d_VerticalHeightPrefixSum[idx] = shared[threadIdx.x];
}
__global__ void SetDrawTriangleHeightTriangleIdx(int* d_VerticalHeightPrefixSum, int* d_DrawTriangleHeightTriangleIdx)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= DrawTriangleCount)	return;

	int start = idx == 0 ? 0 : d_VerticalHeightPrefixSum[idx - 1];
	int end = d_VerticalHeightPrefixSum[idx];
	for (int i = start; i < end; i++)
	{
		d_DrawTriangleHeightTriangleIdx[i] = idx;
	}
}
__global__ void SetDrawTriangleHeight(int* d_VerticalBottom, int* d_VerticalHeightPrefixSum, int* d_DrawTriangleHeightTriangleIdx, int* d_DrawTriangleHeight)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= DrawTriangleHeightCount)	return;

	int triangleIdx = d_DrawTriangleHeightTriangleIdx[idx];
	int beforeSum = triangleIdx == 0 ? 0 : d_VerticalHeightPrefixSum[triangleIdx - 1];

	d_DrawTriangleHeight[idx] = d_VerticalBottom[triangleIdx] + idx - beforeSum;
}
__global__ void SetDrawTriangleHeightInfo(float4* d_DrawVertexPos, float2* d_DrawVertexUV, int* d_DrawTriangleHeightTriangleIdx, int* d_DrawTriangleHeight, 
	int* d_DrawTriangleHeightLeft, int* d_DrawTriangleHeightRight, float2* d_DTHLeftUV, float2* d_DTHRightUV, float* d_DTHLeftDepth, float* d_DTHRightDepth, 
	int* d_FragmentCountPrefixSum, float* d_Log)
{
	extern __shared__ int shared[];

	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= DrawTriangleHeightCount)	return;

	int triangleIdx = d_DrawTriangleHeightTriangleIdx[idx];
	int iHeight = d_DrawTriangleHeight[idx];
	float fHeight = iHeight + 0.5f;

	int vIdx = triangleIdx * 3;
	float4 va = d_DrawVertexPos[vIdx];
	float4 vb = d_DrawVertexPos[vIdx + 1];
	float4 vc = d_DrawVertexPos[vIdx + 2];
	float4 firstv, secondv;
	float2 a, b;
	a.x = vc.x - va.x;
	a.y = vc.y - va.y;
	b.x = vc.x - vb.x;
	b.y = vc.y - vb.y;
	float aDota = a.x * a.x + a.y * a.y;
	float bDotb = b.x * b.x + b.y * b.y;
	float aDotb = a.x * b.x + a.y * b.y;
	float inv_a_w = 1 / va.w;
	float inv_b_w = 1 / vb.w;

	float2 pos, d, leftUV, rightUV;
	float leftDepth, rightDepth;
	float dDota, dDotb, denominator, inv_denominator, s, t, oneMinusST, inv_wa_t, inv_wb_s, inv_wc_oMST, inv_w_total;

	float2 uva = d_DrawVertexUV[vIdx];
	float2 uvb = d_DrawVertexUV[vIdx + 1];
	float2 uvc = d_DrawVertexUV[vIdx + 2];

	float horvalue[2];
	int horvalueIdx = 0;
	if (va.y != vb.y)
	{
		if (va.y < vb.y)
		{
			firstv = va;
			secondv = vb;
		}
		else
		{
			firstv = vb;
			secondv = va;
		}
		if (firstv.y <= fHeight && secondv.y >= fHeight)
		{
			float t = (fHeight - secondv.y) / (firstv.y - secondv.y);
			horvalue[horvalueIdx] = firstv.x * t + secondv.x * (1 - t);
			horvalueIdx++;
		}
	}
	if (vb.y != vc.y)
	{
		if (vb.y < vc.y)
		{
			firstv = vb;
			secondv = vc;
		}
		else
		{
			firstv = vc;
			secondv = vb;
		}
		if (firstv.y <= fHeight && secondv.y >= fHeight)
		{
			float t = (fHeight - secondv.y) / (firstv.y - secondv.y);
			horvalue[horvalueIdx] = firstv.x * t + secondv.x * (1 - t);
			horvalueIdx++;
		}
	}

	if (vc.y != va.y)
	{
		if (vc.y < va.y)
		{
			firstv = vc;
			secondv = va;
		}
		else
		{
			firstv = va;
			secondv = vc;
		}
		if (firstv.y <= fHeight && secondv.y >= fHeight)
		{
			float t = (fHeight - secondv.y) / (firstv.y - secondv.y);
			horvalue[horvalueIdx] = firstv.x * t + secondv.x * (1 - t);
			horvalueIdx++;
		}
	}

	float fRight = max(horvalue[0], horvalue[1]);
	float fLeft = min(horvalue[0], horvalue[1]);
	int iRight, iLeft;

	int iValue = (int)fLeft;
	int addValue = fLeft > iValue + 0.5f ? 1 : 0;
	iLeft = iValue + addValue;
	d_DrawTriangleHeightLeft[idx] = iLeft;

	iValue = (int)fRight;
	addValue = fRight > iValue + 0.5f ? 0 : -1;
	iRight = iValue + addValue;
	d_DrawTriangleHeightRight[idx] = iRight;

	int width = iRight - iLeft + 1;

	pos.x = fLeft;
	pos.y = fHeight;

	d.x = vc.x - pos.x;
	d.y = vc.y - pos.y;

	dDota = d.x * a.x + d.y * a.y;
	dDotb = d.x * b.x + d.y * b.y;
	denominator = aDotb * aDotb - aDota * bDotb;

	inv_denominator = 1 / denominator;
	s = (dDota * aDotb - dDotb * aDota) * inv_denominator;
	t = (dDotb * aDotb - dDota * bDotb) * inv_denominator;
	oneMinusST = 1 - s - t;

	leftDepth = va.z * t + vb.z * s + vc.z * oneMinusST;
	d_DTHLeftDepth[idx] = leftDepth;

	inv_wa_t = t * inv_a_w;
	inv_wb_s = s * inv_b_w;
	inv_wc_oMST = oneMinusST / vc.w;

	inv_w_total = 1 / (inv_wa_t + inv_wb_s + inv_wc_oMST);
	leftUV.x = (uva.x * inv_wa_t + uvb.x * inv_wb_s + uvc.x * inv_wc_oMST) * inv_w_total;
	leftUV.y = (uva.y * inv_wa_t + uvb.y * inv_wb_s + uvc.y * inv_wc_oMST) * inv_w_total;
	d_DTHLeftUV[idx] = leftUV;


	pos.x = fRight;

	d.x = vc.x - pos.x;

	dDota = d.x * a.x + d.y * a.y;
	dDotb = d.x * b.x + d.y * b.y;
	denominator = aDotb * aDotb - aDota * bDotb;

	inv_denominator = 1 / denominator;
	s = (dDota * aDotb - dDotb * aDota) * inv_denominator;
	t = (dDotb * aDotb - dDota * bDotb) * inv_denominator;
	oneMinusST = 1 - s - t;

	rightDepth = va.z * t + vb.z * s + vc.z * oneMinusST;
	d_DTHRightDepth[idx] = rightDepth;

	inv_wa_t = t * inv_a_w;
	inv_wb_s = s * inv_b_w;
	inv_wc_oMST = oneMinusST / vc.w;

	inv_w_total = 1 / (inv_wa_t + inv_wb_s + inv_wc_oMST);
	rightUV.x = (uva.x * inv_wa_t + uvb.x * inv_wb_s + uvc.x * inv_wc_oMST) * inv_w_total;
	rightUV.y = (uva.y * inv_wa_t + uvb.y * inv_wb_s + uvc.y * inv_wc_oMST) * inv_w_total;
	d_DTHRightUV[idx] = rightUV;
	shared[threadIdx.x] = width;
	for (int stride = 1; stride < blockDim.x; stride <<= 1)
	{
		__syncthreads();
		if (threadIdx.x >= stride)
		{
			shared[threadIdx.x] += shared[threadIdx.x - stride];
		}
	}

	d_FragmentCountPrefixSum[idx] = shared[threadIdx.x];
}
__global__ void SetFragmentDTHIdx(int* d_FragmentCountPrefixSum, int* d_FragmentDTHIdx)
{
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= DrawTriangleHeightCount)	return;

	int start = idx == 0 ? 0 : d_FragmentCountPrefixSum[idx - 1];
	int end = d_FragmentCountPrefixSum[idx];
	for (int i = start; i < end; i++)
	{
		d_FragmentDTHIdx[i] = idx;
	}
}
__global__ void SetBuffer(int* d_FragmentDTHIdx, int* d_FragmentCountPrefixSum, int* d_DrawTriangleHeight, int* d_DrawTriangleHeightLeft, int* d_DrawTriangleHeightRight, float2* d_DTHLeftUV, float2* d_DTHRightUV, float* d_DTHLeftDepth, float* d_DTHRightDepth,
	d_Color32* d_TexBuffer, int* d_TexWidth, int* d_TexHeight, d_Color32* d_ColorBuffer, float* d_DepthBuffer, float* d_Log)
{
	//return; // 45
	int idx = (gridDim.x * blockIdx.y + blockIdx.x) * blockDim.x + threadIdx.x;
	if (idx >= FragmentCount)	return;

	int DTHIdx = d_FragmentDTHIdx[idx];
	int beforePrefixSum = DTHIdx == 0 ? 0 : d_FragmentCountPrefixSum[DTHIdx - 1];
	int tex_xIdx, tex_yIdx, texIdx, bufferIdx;

	int texWidth = d_TexWidth[0];
	int texHeight = d_TexHeight[0];

	int iLeft = d_DrawTriangleHeightLeft[DTHIdx];
	int iRight = d_DrawTriangleHeightRight[DTHIdx];
	float2 leftUV = d_DTHLeftUV[DTHIdx];
	float2 rightUV = d_DTHRightUV[DTHIdx];
	float leftDepth = d_DTHLeftDepth[DTHIdx];
	float rightDepth = d_DTHRightDepth[DTHIdx];
	int iHeight = d_DrawTriangleHeight[DTHIdx];
	int iX = iLeft + idx - beforePrefixSum;

	int n = iRight - iLeft;
	int k = iX - iLeft;
	float ratio = (k * leftDepth) / (k * leftDepth + (n - k) * rightDepth);
	float curDepth = leftDepth + (rightDepth - leftDepth) * ratio;
	bufferIdx = ScreenSize.x * iHeight + iX;
	
	atomicMin(&d_DepthBuffer[bufferIdx], curDepth);

	if (d_DepthBuffer[bufferIdx] == curDepth)
	{
		float2 curUV;
		curUV.x = leftUV.x + (rightUV.x - leftUV.x) * ratio;
		curUV.y = leftUV.y + (rightUV.y - leftUV.y) * ratio;
		tex_xIdx = max(0, min((int)(curUV.x * texWidth), texWidth - 1));
		tex_yIdx = max(0, min((int)(curUV.y * texHeight), texHeight - 1));
		texIdx = tex_xIdx + tex_yIdx * texWidth;
		d_ColorBuffer[bufferIdx] = d_TexBuffer[texIdx];
	}
	

	//return; // 124 ~ 127
}


void CU_Culling_ScanLine_SetBuffer()
{
	cudaStream_t S_CullingScanLineRange;
	cudaStreamCreate(&S_CullingScanLineRange);
	dim3 block;
	int thread;

	Block_Thread_Size(h_TriangleCountAfterClip, block, thread);
	Culling << <block, thread, thread * sizeof(int), S_CullingScanLineRange >> > (d_VertexPos, d_isDraw, d_DrawTriangleIdxPrefixSum);
	int blockCount = block.x * block.y;
	if (blockCount > 1)
	{
		CU_KoggeStoneScan_Block(d_DrawTriangleIdxPrefixSum, block, thread, h_TriangleCountAfterClip, blockCount, S_CullingScanLineRange);
	}
	cudaMemcpy(&h_DrawTriangleCount, d_DrawTriangleIdxPrefixSum + h_TriangleCountAfterClip - 1, sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpyToSymbolAsync(DrawTriangleCount, d_DrawTriangleIdxPrefixSum + h_TriangleCountAfterClip - 1, sizeof(int), 0, cudaMemcpyDeviceToDevice, S_CullingScanLineRange);
	static int DrawTriangleArrayCount = 0;
	if (h_DrawTriangleCount > DrawTriangleArrayCount)
	{
		if (DrawTriangleArrayCount != 0)
		{
			cudaFree(d_DrawVertexPos);
			cudaFree(d_DrawVertexUV);
			cudaFree(d_VerticalBottom);
			cudaFree(d_VerticalHeightPrefixSum);
			cudaFree(d_DrawTriangleIdxPrefixSumCom);
		}
		DrawTriangleArrayCount = h_DrawTriangleCount + (h_DrawTriangleCount >> 2);
		cudaMalloc(&d_DrawVertexPos, sizeof(float4) * DrawTriangleArrayCount * 3);
		cudaMalloc(&d_DrawVertexUV, sizeof(float2) * DrawTriangleArrayCount * 3);
		cudaMalloc(&d_VerticalBottom, sizeof(int) * DrawTriangleArrayCount);
		cudaMalloc(&d_VerticalHeightPrefixSum, sizeof(int) * DrawTriangleArrayCount);
		cudaMalloc(&d_DrawTriangleIdxPrefixSumCom, sizeof(int) * DrawTriangleArrayCount);
	}
	SetDrawTriangleIdxPrefixCom << <block, thread, 0, S_CullingScanLineRange >> > (d_isDraw, d_DrawTriangleIdxPrefixSum, d_DrawTriangleIdxPrefixSumCom);
	Block_Thread_Size(h_DrawTriangleCount, block, thread);
	DrawVertexSetting_GetVerticalRange << <block, thread, thread * sizeof(int), S_CullingScanLineRange >> > (d_VertexPos, d_VertexUV, d_DrawVertexPos, d_DrawVertexUV, d_DrawTriangleIdxPrefixSumCom, d_VerticalBottom, d_VerticalHeightPrefixSum);
	
	blockCount = block.x * block.y;
	if (blockCount > 1)
	{
		CU_KoggeStoneScan_Block(d_VerticalHeightPrefixSum, block, thread, h_DrawTriangleCount, blockCount, S_CullingScanLineRange);
	}
	cudaMemcpyToSymbolAsync(DrawTriangleHeightCount, d_VerticalHeightPrefixSum + h_DrawTriangleCount - 1, sizeof(int), 0, cudaMemcpyDeviceToDevice, S_CullingScanLineRange);
	cudaMemcpy(&h_DrawTriangleHeightCount, d_VerticalHeightPrefixSum + h_DrawTriangleCount - 1, sizeof(int), cudaMemcpyDeviceToHost);

	//return; //18~20

	static int DrawTriangleHeightArrayCount = 0;
	if (h_DrawTriangleHeightCount > DrawTriangleHeightArrayCount)
	{
		if (DrawTriangleHeightArrayCount != 0)
		{
			cudaFree(d_DrawTriangleHeightTriangleIdx);
			cudaFree(d_DrawTriangleHeight);
			cudaFree(d_DrawTriangleHeightLeft);
			cudaFree(d_DrawTriangleHeightRight);
			cudaFree(d_DTHLeftUV);
			cudaFree(d_DTHRightUV);
			cudaFree(d_DTHLeftDepth);
			cudaFree(d_DTHRightDepth);
			cudaFree(d_FragmentCountPrefixSum);

		}
		DrawTriangleHeightArrayCount = h_DrawTriangleHeightCount + (h_DrawTriangleHeightCount >> 2);
		cudaMalloc(&d_DrawTriangleHeightTriangleIdx, sizeof(int) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DrawTriangleHeight, sizeof(int) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DrawTriangleHeightLeft, sizeof(int) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DrawTriangleHeightRight, sizeof(int) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DTHLeftUV, sizeof(float2) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DTHRightUV, sizeof(float2) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DTHLeftDepth, sizeof(float) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_DTHRightDepth, sizeof(float) * DrawTriangleHeightArrayCount);
		cudaMalloc(&d_FragmentCountPrefixSum, sizeof(int) * DrawTriangleHeightArrayCount);
	}

	SetDrawTriangleHeightTriangleIdx << <block, thread, 0, S_CullingScanLineRange >> > (d_VerticalHeightPrefixSum, d_DrawTriangleHeightTriangleIdx);

	Block_Thread_Size(h_DrawTriangleHeightCount, block, thread);
	SetDrawTriangleHeight << <block, thread, 0, S_CullingScanLineRange >> > (d_VerticalBottom, d_VerticalHeightPrefixSum, d_DrawTriangleHeightTriangleIdx, d_DrawTriangleHeight);
	SetDrawTriangleHeightInfo << <block, thread, thread * sizeof(int), S_CullingScanLineRange >> > (d_DrawVertexPos, d_DrawVertexUV, d_DrawTriangleHeightTriangleIdx, d_DrawTriangleHeight,
		d_DrawTriangleHeightLeft, d_DrawTriangleHeightRight, d_DTHLeftUV, d_DTHRightUV, d_DTHLeftDepth, d_DTHRightDepth, d_FragmentCountPrefixSum, d_Log);

	blockCount = block.x * block.y;
	if (blockCount > 1)
	{
		CU_KoggeStoneScan_Block(d_FragmentCountPrefixSum, block, thread, h_DrawTriangleHeightCount, blockCount, S_CullingScanLineRange);
	}

	cudaMemcpyToSymbolAsync(FragmentCount, d_FragmentCountPrefixSum + h_DrawTriangleHeightCount - 1, sizeof(int), 0, cudaMemcpyDeviceToDevice, S_CullingScanLineRange);
	cudaMemcpy(&h_FragmentCount, d_FragmentCountPrefixSum + h_DrawTriangleHeightCount - 1, sizeof(int), cudaMemcpyDeviceToHost);
	//return; //32~34
	static int FragmentArrayCount = 0;
	if (h_FragmentCount > FragmentArrayCount)
	{
		if (FragmentArrayCount != 0)
		{
			cudaFree(d_FragmentDTHIdx);
		}
		FragmentArrayCount = h_FragmentCount + (h_FragmentCount >> 2);
		cudaMalloc(&d_FragmentDTHIdx, sizeof(int) * FragmentArrayCount);
	}
	//return; //35
	SetFragmentDTHIdx << <block, thread, 0, S_CullingScanLineRange >> > (d_FragmentCountPrefixSum, d_FragmentDTHIdx);
	Block_Thread_Size(h_FragmentCount, block, thread);
	//return;
	SetBuffer << <block, thread, 0, S_CullingScanLineRange >> > (d_FragmentDTHIdx, d_FragmentCountPrefixSum, d_DrawTriangleHeight, d_DrawTriangleHeightLeft, d_DrawTriangleHeightRight, d_DTHLeftUV, d_DTHRightUV, d_DTHLeftDepth, d_DTHRightDepth,
		d_TexBuffer, d_TexWidth, d_TexHeight, d_ColorBuffer, d_DepthBuffer, d_Log);
	cudaStreamSynchronize(S_CullingScanLineRange);
	cudaStreamDestroy(S_CullingScanLineRange);
	cudaMemcpy(h_Log, d_Log, sizeof(float) * 100, cudaMemcpyDeviceToHost);
	float log0 = h_Log[0];
	float log1 = h_Log[1];
	float log2 = h_Log[2];
	float log3 = h_Log[3];
	float log4 = h_Log[4];
	float log5 = h_Log[5];
	int a = 0;
}


//삼각형의 사각범위 구하기
//모든 사각범위
//

d_Matrix4x4* d_LocalToWorldMat;
Matrix4x4* h_LocalToWorldMat;
void CU_DrawCall(int materialIdx, vector<GameObject*> v_pGo_Visible)
{
	float startTime = clock();
	h_GoCount = v_pGo_Visible.size();
	static int GoArrayCount = 0;
	if (GoArrayCount < h_GoCount)
	{
		if (GoArrayCount != 0)
		{
			cudaFree(d_MeshIdx);
			delete[] h_MeshIdx;
			cudaFree(d_BCR);
			delete[] h_BCR;
			cudaFree(d_LocalToWorldMat);
			delete[] h_LocalToWorldMat;
			cudaFree(d_LocalToNDCMat);
		}
		GoArrayCount = h_GoCount + (h_GoCount >> 2);
		cudaMalloc(&d_MeshIdx, sizeof(int) * GoArrayCount);
		h_MeshIdx = new int[GoArrayCount];
		cudaMalloc(&d_BCR, sizeof(int) * GoArrayCount);
		h_BCR = new int[GoArrayCount];
		cudaMalloc(&d_LocalToWorldMat, sizeof(d_Matrix4x4) * GoArrayCount);
		h_LocalToWorldMat = new Matrix4x4[GoArrayCount];
		cudaMalloc(&d_LocalToNDCMat, sizeof(d_Matrix4x4) * GoArrayCount);
	}

	cudaMemcpyToSymbolAsync(MaterialIdx, &materialIdx, sizeof(int), 0, cudaMemcpyHostToDevice);
	cudaMemcpyToSymbolAsync(GoCount, &h_GoCount, sizeof(int), 0, cudaMemcpyHostToDevice);
	for (int i = 0; i < h_GoCount; i++)
	{
		h_MeshIdx[i] = Engine::PInstance->GetMeshIdx(v_pGo_Visible[i]->GetMeshKey());
		h_LocalToWorldMat[i] = v_pGo_Visible[i]->GetTransform().GetModelingMatrix();
	}
	cudaMemcpyAsync(d_MeshIdx, h_MeshIdx, sizeof(int) * h_GoCount, cudaMemcpyHostToDevice);
	cudaMemcpyAsync(d_LocalToWorldMat, h_LocalToWorldMat, sizeof(d_Matrix4x4) * h_GoCount, cudaMemcpyHostToDevice);
	CU_ConstantMatMulMat(h_GoCount, h_WorldToNDCMat, d_LocalToWorldMat, d_LocalToNDCMat);
	Engine::PInstance->_TimeLog.CU_DrawCallSetting = clock() - startTime;
	startTime = clock();
	for (int i = 0; i < 1; i++)
	{
		CU_BoundCheckBox(); //h_BCR에 outside = 4, intersect = 2, inside = 1 구분
	}
	Engine::PInstance->_TimeLog.CU_BoundCheckBox = clock() - startTime;
	startTime = clock();
	for (int i = 0; i < 1; i++)
	{
		CU_VertexArraySetting();
	}
	Engine::PInstance->_TimeLog.CU_VertexArraySetting = clock() - startTime;
	startTime = clock();
	for (int i = 0; i < 1; i++)
	{
		CU_Culling_ScanLine_SetBuffer();
	}
	Engine::PInstance->_TimeLog.CU_Culling_ScanLine_SetBuffer = clock() - startTime;
	startTime = clock();
}

#pragma endregion


void CU_BufferCpy_Free(Color32* colorBuffer, float* depthBuffer)
{
	cudaMemcpy(colorBuffer, d_ColorBuffer, sizeof(d_Color32) * h_ScreenSize, cudaMemcpyDeviceToHost);
	cudaMemcpy(depthBuffer, d_DepthBuffer, sizeof(float) * h_ScreenSize, cudaMemcpyDeviceToHost);



}
