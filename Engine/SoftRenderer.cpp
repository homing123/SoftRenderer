#include "EngineLib.h"

extern void CU_ChangeScreenSize(const ScreenPoint& screenSize);
extern void CU_BufferInit(Color32*& colorBuffer, float*& depthBuffer);
extern void CU_BufferFree(Color32*& colorBuffer, float*& depthBuffer);
extern void CU_BG_Depth_CamInfoSetting(const Color32& bgColor, const ScreenPoint& screenSize, Camera* pMainCam);
extern void CU_BufferCpy_Free(Color32* colorBuffer, float* depthBuffer);
extern void CU_DrawCall(int materialIdx, vector<GameObject*> v_pGo_Visible);
SoftRenderer::SoftRenderer(const ScreenPoint& size, const HWND& hwnd)
{

	_ScreenSize = size;
	_hWnd = hwnd;

	Init();
}
void SoftRenderer::SetScreenSize(const ScreenPoint& screensize) {

	_ScreenSize = screensize;
	CU_ChangeScreenSize(_ScreenSize);
	CU_BufferFree(_ColorBuffer, _DepthBuffer);
	DeleteObject(_Bitmap);
	BITMAPINFO bmi;
	memset(&bmi, 0, sizeof(BITMAPINFO));
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = _ScreenSize.X;
	bmi.bmiHeader.biHeight = -_ScreenSize.Y; // 음수로 설정하여 픽셀 배열이 상단에서 시작하도록 함
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32; // 32비트 컬러 사용
	bmi.bmiHeader.biCompression = BI_RGB;

	_Bitmap = CreateDIBSection(_MemoryDC, &bmi, DIB_RGB_COLORS, (void**)&_ColorBuffer, NULL, 0);
	if (_Bitmap == NULL) {
		// 생성 실패 처리
		return;
	}

	_BitmapOld = (HBITMAP)SelectObject(_MemoryDC, _Bitmap);
	CU_BufferInit(_ColorBuffer, _DepthBuffer);

}

void SoftRenderer::Init()
{
	_ScreenDC = GetDC(_hWnd);
	CU_ChangeScreenSize(_ScreenSize);

	//ScreenDC와 호환지만 화면에 그려지지않는 다른 DC를 만드는 함수
	_MemoryDC = CreateCompatibleDC(_ScreenDC);
	BITMAPINFO bmi;
	memset(&bmi, 0, sizeof(BITMAPINFO));
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = _ScreenSize.X;
	bmi.bmiHeader.biHeight = -_ScreenSize.Y; // 음수로 설정하여 픽셀 배열이 상단에서 시작하도록 함
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 32; // 32비트 컬러 사용
	bmi.bmiHeader.biCompression = BI_RGB;

	_Bitmap = CreateDIBSection(_MemoryDC, &bmi, DIB_RGB_COLORS, (void**)&_ColorBuffer, NULL, 0);

	if (_Bitmap == NULL) {
		// 생성 실패 처리
		return;
	}

	_BitmapOld = (HBITMAP)SelectObject(_MemoryDC, _Bitmap);
	ChangeBGColor(Color32::White);
	CU_BufferInit(_ColorBuffer, _DepthBuffer);

}

int OutsideCount = 0;
int InsideCount = 0;
int IntersectCount = 0;

void SoftRenderer::RenderGPU()
{
	float startTime = clock();
	CU_BG_Depth_CamInfoSetting(_BGColor, _ScreenSize, &Engine::PInstance->GetMainCam());
	Engine::PInstance->_TimeLog.CU_BG_Depth_CamInfoSetting = clock() - startTime;
	CU_DrawCall(0, Engine::PInstance->GetVisibleGO());
	startTime = clock();
	CU_BufferCpy_Free(_ColorBuffer, _DepthBuffer);
	Engine::PInstance->_TimeLog.CU_BufferCopy = clock() - startTime;
	startTime = clock();

	// 그리기 작업 수행
	BitBlt(_ScreenDC, 0, 0, _ScreenSize.X, _ScreenSize.Y, _MemoryDC, 0, 0, SRCCOPY); //복사 및 출력

	//텍스트 출력
	DrawUI();

}


#pragma region Draw
void SoftRenderer::ChangeBGColor(const Color32& color)
{
	_BGColor = color;
}
#pragma endregion
#pragma region  UI
void SoftRenderer::DrawUI()
{
	const auto& texts = Engine::PInstance->GetTexts();
	int size = texts.size();
	for (int i = 0; i < size; i++)
	{
		Text text = texts[i];
		SetTextColor(_ScreenDC, RGB(text._Color.R, text._Color.G, text._Color.B));
		LPCWSTR str = text._String.c_str();
		TextOut(_ScreenDC, text._Pos.X, text._Pos.Y, str, lstrlen(str));
	}
}
#pragma endregion


