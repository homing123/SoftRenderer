#pragma once

#include "EngineUtil.h"


enum class DrawMode
{
	Color,
	Texture,
	Wireframe,
	DepthBuffer,
};

enum class TriangleRes
{
	Origin,
	ScanLine,
};
enum class Hardware
{
	CPU,
	CPU_Thread,
	GPU_CUDA
};

class SoftRenderer
{
public:
	SoftRenderer() = default;
	SoftRenderer(const ScreenPoint& psize, const HWND& hwnd);

	void ChangeBGColor(const Color32& color);
	void RenderGPU();
	void SetScreenSize(const ScreenPoint& screensize);

	VertexInfo _VertexInfo;
	DrawInfo _DrawInfo;

	FORCEINLINE constexpr ScreenPoint GetScreenSize() const
	{
		return _ScreenSize;
	}
	FORCEINLINE constexpr int GetScreenBufferSize() const
	{
		return _ScreenSize.X * _ScreenSize.Y;
	}

private:
	void Init();

	void DrawUI();

	bool _BackSpaceCulling = true;

	Color32 _BGColor;

	ScreenPoint _ScreenSize;
	HWND _hWnd;

	HBITMAP _Bitmap, _BitmapOld;

	HDC _ScreenDC, _MemoryDC;

	Color32* _ColorBuffer;
	float* _DepthBuffer;
};


//오른쪽 위 픽셀을 그림