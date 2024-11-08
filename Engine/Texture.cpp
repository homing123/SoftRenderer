
#include "Texture.h"

Texture::Texture(const wstring& filePath)
{
	Bitmap* image = Bitmap::FromFile(filePath.c_str());
	_Width = image->GetWidth();
	_Height = image->GetHeight();
	_Buffer = new Color32[_Width * _Height];

	Gdiplus::Color* pColor = new Gdiplus::Color();
	for (UINT y = 0; y < _Height; y++)
	{
		for (UINT x = 0; x < _Width; x++)
		{
			image->GetPixel(x, _Height - 1 - y, pColor);
			_Buffer[y * _Width + x] = Color32(pColor->GetR(), pColor->GetG(), pColor->GetB(), pColor->GetA());
		}
	}
	delete pColor;
	delete image;
}
int releaseCount = 0;
void Texture::Release()
{
	releaseCount++;
	_Width = 0;
	_Height = 0;
	delete[] _Buffer;
}

Color32 Texture::GetPixel(const Vector2& uv) const
{
	int x = 0;
	int y = 0;

	if (uv.X > 0 && uv.X < 1)
	{
		x = (int)(uv.X * _Width);
	}
	else
	{
		if (uv.X <= 0)
		{
			x = 0;
		}
		else if (uv.X >= 1)
		{
			x = _Width - 1;
		}
	}

	if (uv.Y > 0 && uv.Y < 1)
	{
		y = (int)(uv.Y * _Height);
	}
	else
	{
		if (uv.Y <= 0)
		{
			y = 0;
		}
		else if (uv.Y >= 1)
		{
			y = _Height - 1;
		}
	}

	return _Buffer[x + _Width * y];
}