
#include "InputManager.h"

static const Key All[] = { PG_Up,PG_Down, Arrow_Up, Arrow_Down, Arrow_Left,Arrow_Right, NP0, NP1, NP2, NP3, NP4, NP5, NP6, NP7, NP8, NP9,A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z };

InputManager::InputManager()
{
	for (const auto e : All)
	{
		_isDown[e] = false;
		_InputState[e] = None;
	}
}

void InputManager::ChangeisDown(const Key& key, const bool isDown)
{
	if (_isDown.find(key) != _isDown.end())
	{
		_isDown[key] = isDown;
	}
}
void InputManager::UpdateState() 
{
	//WndProc 이 1번불릴때 update 함수는 5번불림(100fps기준) 이 때문에 기존로직으로는 KeyDown이 5번씩 불리는 현상 발생

	for (const auto e : All)
	{
		switch (_InputState[e])
		{
		case None:
			if (_isDown[e])
			{
				_InputState[e] = Down;
			}
			break;
		case Up:
			_InputState[e] = _isDown[e] ? Down : None;
			break;
		case Down:		
			_InputState[e] = _isDown[e] ? Pushing : Up;	
			break;
		case Pushing:
			if (_isDown[e] == false)
			{
				_InputState[e] = Up;
			}
			break;
		}
	}
}

bool InputManager::isKey(const Key& key)
{
	return _InputState[key] == Pushing || _InputState[key] == Down;
}
bool InputManager::isKeyDown(const Key& key)
{
	return _InputState[key] == Down;
}
bool InputManager::isKeyUp(const Key& key)
{
	return _InputState[key] == Up;
}

void InputManager::AddLog(const int& id)
{
	_Log.push_back(id);
}

void InputManager::Loging()
{
	int idx = 0;
	for (auto const& i : _Log)
	{
		idx++;
	}
}