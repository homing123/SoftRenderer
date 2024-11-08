#pragma once
#include <time.h>

class TimeUtil
{
public:
	static float GetDeltaTime();
private:
	static float Previous;
};
