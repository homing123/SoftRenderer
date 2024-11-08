#include "TimeUtil.h"

float TimeUtil::Previous = 0.f;

float TimeUtil::GetDeltaTime() {
	clock_t cur = clock();
	float deltaTime = (cur - TimeUtil::Previous) / CLOCKS_PER_SEC;
	Previous = (float)cur;
	return deltaTime;
}