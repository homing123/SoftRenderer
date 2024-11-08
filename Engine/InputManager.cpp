
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
	//WndProc �� 1���Ҹ��� update �Լ��� 5���Ҹ�(100fps����) �� ������ �����������δ� KeyDown�� 5���� �Ҹ��� ���� �߻�

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