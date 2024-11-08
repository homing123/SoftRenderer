#include "MathLib.h"

using namespace HM;

//std::mt19937 Math::Generator((unsigned int)time(NULL));
std::mt19937 Math::Generator(0);

const LinearColor LinearColor::Error(1.f, 0.f, 1.f, 1.f);
const LinearColor LinearColor::White(1.f, 1.f, 1.f, 1.f);
const LinearColor LinearColor::Black(0.f, 0.f, 0.f, 1.f);
const LinearColor LinearColor::Gray(0.5f, 0.5f, 0.5f, 1.f);
const LinearColor LinearColor::Red(1.f, 0.f, 0.f, 1.f);
const LinearColor LinearColor::Green(0.f, 1.f, 0.f, 1.f);
const LinearColor LinearColor::Blue(0.f, 0.f, 1.f, 1.f);
const LinearColor LinearColor::Yellow(1.f, 1.f, 0.f, 1.f);
const LinearColor LinearColor::Magenta(1.f, 0.f, 1.f, 1.f);
const LinearColor LinearColor::Empty(0.f, 0.f, 0.f, 0.f);

const Color32 Color32::Error(255, 0, 255, 255);
const Color32 Color32::White(255, 255, 255, 255);
const Color32 Color32::Black(0, 0, 0, 255);
const Color32 Color32::Gray(128, 128, 128, 255);
const Color32 Color32::Red(255, 0, 0, 255);
const Color32 Color32::Green(0, 255, 0, 255);
const Color32 Color32::Blue(0, 0, 255, 255);
const Color32 Color32::Yellow(255, 255, 0, 255);
const Color32 Color32::Cyan(0, 255, 255, 255);
const Color32 Color32::Magenta(255, 0, 255, 255);


const Vector2 Vector2::Zero(0, 0);
const Vector2 Vector2::One(1, 1);
const Vector2 Vector2::UnitX(1, 0);
const Vector2 Vector2::UnitY(0, 1);

const Vector3 Vector3::Zero(0, 0, 0);
const Vector3 Vector3::One(1, 1, 1);
const Vector3 Vector3::UnitX(1, 0, 0);
const Vector3 Vector3::UnitY(0, 1, 0);
const Vector3 Vector3::UnitZ(0, 0, 1);

const Vector4 Vector4::Zero(0, 0, 0, 0);
const Vector4 Vector4::One(1, 1, 1, 1);
const Vector4 Vector4::UnitX(1, 0, 0, 0);
const Vector4 Vector4::UnitY(1, 0, 0, 0);
const Vector4 Vector4::UnitZ(1, 0, 0, 0);
const Vector4 Vector4::UnitW(1, 0, 0, 0);

const Matrix2x2 Matrix2x2::Identity = Matrix2x2{ Vector2::UnitX, Vector2::UnitY };
const Matrix3x3 Matrix3x3::Identity = Matrix3x3{ Vector3::UnitX, Vector3::UnitY, Vector3::UnitZ };
const Matrix4x4 Matrix4x4::Identity = Matrix4x4{ Vector4::UnitX, Vector4::UnitY, Vector4::UnitZ, Vector4::UnitW };
