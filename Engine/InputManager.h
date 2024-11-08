#pragma once

#include "EngineUtil.h"

namespace HM
{
	enum Key
	{
		PG_Up = 0x21,
		PG_Down = 0x22,
		Arrow_Up = 0x26,
		Arrow_Down = 0x28,
		Arrow_Left = 0x25,
		Arrow_Right = 0x27,
		NP0 = 0x30,
		NP1 = 0x31,
		NP2 = 0x32,
		NP3 = 0x33,
		NP4 = 0x34,
		NP5 = 0x35,
		NP6 = 0x36,
		NP7 = 0x37,
		NP8 = 0x38,
		NP9 = 0x39,
		A = 0x41,
		B = 0x42,
		C = 0x43,
		D = 0x44,
		E = 0x45,
		F = 0x46,
		G = 0x47,
		H = 0x48,
		I = 0x49,
		J = 0x4A,
		K = 0x4B,
		L = 0x4C,
		M = 0x4D,
		N = 0x4E,
		O = 0x4F,
		P = 0x50,
		Q = 0x51,
		R = 0x52,
		S = 0x53,
		T = 0x54,
		U = 0x55,
		V = 0x56,
		W = 0x57,
		X = 0x58,
		Y = 0x59,
		Z = 0x5A
	};


	enum KeyState
	{
		None,
		Up,
		Down,
		Pushing,
	};

	class InputManager
	{
	public:
		InputManager();

		map<Key, bool> _isDown;
		map<Key, KeyState> _InputState;

		void ChangeisDown(const Key& key, const bool isDown);
		void UpdateState();

		bool isKeyDown(const Key& key);
		bool isKeyUp(const Key& key);
		bool isKey(const Key& key);

		void AddLog(const int& id);
		void Loging();
		std::vector<int> _Log;
	};
}