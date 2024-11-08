#pragma once
#include "EngineUtil.h"
#include <ole2.h>
#include <gdiplus.h>
#include <wchar.h>
#pragma comment(lib, "Gdiplus.lib")

using namespace Gdiplus;
const size_t ErrorTextureKey = std::hash<std::string>()("Error");
const wstring TextureFolderPath = L"C:\\Users\\k9996\\Desktop\\Renderer\\HMRenderer\\Resource\\";
const std::size_t TexKey_FaceCube = std::hash<std::wstring>()(TextureFolderPath + L"Face_Cube.png");
const std::size_t TexKey_RGBGrid = std::hash<std::wstring>()(TextureFolderPath + L"RGBGrid.png");
namespace HM
{
	class Texture
	{
	public:
		Texture() = default;
		Texture(const wstring& filePath);
		~Texture() {
			Release();
		}

		void Release();
		const int GetWidth() const { return _Width; }
		const int GetHeight() const { return _Height; }
		const int GetSize() const { return _Width * _Height; }
		Color32 GetPixel(const Vector2& uv)const;
		const Color32* GetBuffer() { return _Buffer; }

		Color32* _Buffer;
	private:
		static constexpr BYTE _Channel = 4;
		int _Width = 0;
		int _Height = 0;



	};
}