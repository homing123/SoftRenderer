#pragma once
#include "EngineUtil.h"

namespace HM
{
	//malloc, free 은 재할당이 자주 일어나는경우 좋음 (realloc), caolloc 으로 할당과 동시에 초기화 가능
	//new, delete 는 쓰기 편리함 할당과 동시에 초기화가능
	class DrawInfo
	{
	public:
		DrawInfo() = default;

		void VertexCountInit(const int vSize, const int screenArraySize)
		{
			if (isInit == false)
			{
				isInit = true;
				PixelInfoCount = new int* [ThreadCountWithMainThread];
				PixelInfoStartIdx = new int* [ThreadCountWithMainThread];
			}


			int quotient = screenArraySize / ScreenGridPixelCount;
			int remain = screenArraySize % ScreenGridPixelCount;
			if (remain > 0)
			{
				quotient++;
			}
			ScreenGridCount = quotient;

			if (ScreenGridArrayCount == 0 || ScreenGridArrayCount < ScreenGridCount)
			{
				if (ScreenGridArrayCount != 0)
				{
					for (int i = 0; i < ThreadCountWithMainThread; i++)
					{
						delete[] PixelInfoCount[i];
						delete[] PixelInfoStartIdx[i];
					}
					delete[] GridStartIdx;
				}

				ScreenGridArrayCount = ScreenGridCount;
				for (int i = 0; i < ThreadCountWithMainThread; i++)
				{
					PixelInfoCount[i] = new int[ScreenGridArrayCount];
					PixelInfoStartIdx[i] = new int[ScreenGridArrayCount];
				}
				GridStartIdx = new int[ScreenGridArrayCount + 1]();

			}
			VertexCount = vSize;
			TriangleCount = VertexCount / 3;
			if (VertexArrayCount == 0 || VertexArrayCount < VertexCount)
			{
				if (VertexArrayCount != 0)
				{
					delete[] vertex;
					delete[] isDraw;
					delete[] screenVt2;
					delete[] u;
					delete[] v;
					delete[] inv_w;
					delete[] uDotv;
					delete[] uDotu;
					delete[] vDotv;
					delete[] indenominator;
					delete[] left;
					delete[] right;
					delete[] top;
					delete[] bottom;
				}
				VertexArrayCount = VertexCount * 2;
				TriangleArrayCount = VertexArrayCount / 3;

				vertex = new Vertex[VertexArrayCount];
				isDraw = new bool[TriangleArrayCount];
				screenVt2 = new Vector2[VertexArrayCount];
				u = new Vector2[TriangleArrayCount];
				v = new Vector2[TriangleArrayCount];
				inv_w = new float[VertexArrayCount];
				uDotv = new float[TriangleArrayCount];
				uDotu = new float[TriangleArrayCount];
				vDotv = new float[TriangleArrayCount];
				indenominator = new float[TriangleArrayCount];
				left = new float* [TriangleArrayCount];
				right = new float* [TriangleArrayCount];
				top = new float[TriangleArrayCount];
				bottom = new float[TriangleArrayCount];
			}

		
		}

		void PixelInfoCountInit() 
		{
			int curScreenGridPixelInfoCount = 0;
			for (int i = 0; i < ScreenGridCount; i++)
			{
				for (int j = 0; j < ThreadCountWithMainThread; j++)
				{
					PixelInfoStartIdx[j][i] = curScreenGridPixelInfoCount;
					curScreenGridPixelInfoCount += PixelInfoCount[j][i];
				}
				GridStartIdx[i + 1] = curScreenGridPixelInfoCount;
			}
			TotalPixelInfoCount = curScreenGridPixelInfoCount;
			if (TotalPixelInfoCount > 0 && (TotalPixelInfoArrayCount == 0 || TotalPixelInfoArrayCount < TotalPixelInfoCount))
			{
				if (TotalPixelInfoArrayCount != 0) //첫 할당이 아닌데 사이즈때문에 재할당 해야하는경우
				{
					delete[] PixelInfo;
				}
				TotalPixelInfoArrayCount = TotalPixelInfoCount * 2;
				PixelInfo = new Vt2Idx[TotalPixelInfoArrayCount];
			}
		}
		void Release()
		{
			TriangleCount = 0;
			VertexCount = 0;
			TotalPixelInfoCount = 0;
			pTexture = nullptr;
		}

		bool isInit = false;
		int VertexArrayCount = 0;
		int TriangleArrayCount = 0;
		int TotalPixelInfoArrayCount = 0;

		int ScreenGridArrayCount;
		int ScreenGridCount;
		int TotalPixelInfoCount;
		int TriangleCount;
		int VertexCount;
		int** PixelInfoCount; //[쓰레드idx][위치]
		int** PixelInfoStartIdx; //[쓰레드idx][위치]
		int* GridStartIdx; //[위치]

		Vertex* vertex;
		bool* isDraw;
		Vector2* screenVt2;
		Vector2* u;
		Vector2* v;
		float* inv_w;
		float* uDotv;
		float* uDotu;
		float* vDotv;
		float* indenominator;
		float** left;
		float** right;
		float* top;
		float* bottom;

		Vt2Idx* PixelInfo;

		Texture* pTexture;
	};
}