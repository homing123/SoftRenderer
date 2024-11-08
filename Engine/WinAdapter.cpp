
#include "EngineLib.h"

constexpr int StartScreenWidth = 800;
constexpr int StartScreenHeight = 600;
ScreenPoint StartScreenSize = ScreenPoint(StartScreenWidth, StartScreenHeight);

LRESULT CALLBACK WndProc(HWND hWnd, UINT iMsg, WPARAM wParam, LPARAM lParam)
{
	switch (iMsg)
	{
	case WM_CREATE:
		break;
	case WM_DISPLAYCHANGE:

	break;
	case WM_SIZE:
	{
		if (Engine::PInstance != nullptr) 
		{
			int width = lParam & 0x0000FFFF;
			int height = (lParam >> 16) & 0x0000FFFF;
			Engine::PInstance->ChangeScreenSize(ScreenPoint(width, height));
		}
	}
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_LBUTTONDOWN:
		break;
	case WM_RBUTTONDOWN:
		break;
	case WM_KEYDOWN:
		Engine::PInstance->GetInput().ChangeisDown((Key)wParam, true);
		break;
	case WM_KEYUP:
		Engine::PInstance->GetInput().ChangeisDown((Key)wParam, false);
		break;
	}
	return DefWindowProc(hWnd, iMsg, wParam, lParam);
}


HWND ScreenCreate(HINSTANCE hInstance)
{
	const LPCWSTR lpClassName = TEXT("class");
	const LPCWSTR lpTitleName = TEXT("Title");

	WNDCLASSEX wndc;
	wndc.cbSize = sizeof(wndc);
	wndc.style = CS_HREDRAW | CS_VREDRAW;
	wndc.lpfnWndProc = WndProc;
	wndc.cbClsExtra = NULL;
	wndc.cbWndExtra = NULL;
	wndc.hInstance = hInstance;
	wndc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wndc.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndc.lpszMenuName = NULL;
	wndc.lpszClassName = lpClassName;
	wndc.hIconSm = LoadIcon(NULL, IDI_APPLICATION);

	RegisterClassEx(&wndc);

	HWND hWnd = CreateWindow(
		lpClassName,
		lpTitleName,
		WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT,
		CW_USEDEFAULT,
		StartScreenSize.X,
		StartScreenSize.Y,
		NULL,
		NULL,
		hInstance,
		NULL
	);

	return hWnd;
}

int CALLBACK WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR szCmdLine, int iCmdShow)
{
	HWND hWnd = ScreenCreate(hInstance);
	ShowWindow(hWnd, iCmdShow);
	UpdateWindow(hWnd);
	MSG msg;

	Engine engine = Engine(StartScreenSize, hWnd);
	Engine::PInstance = &engine;

	Engine::PInstance->LoadResource();
	Engine::PInstance->LoadScene();
	Engine::PInstance->Start();
	while (true)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				break;
			}

			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		Engine::PInstance->Cycle();
	}

	return msg.wParam;
}

int main() 
{
	return 0;
}

