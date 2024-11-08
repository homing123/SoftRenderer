#pragma once

#include "MathLib.h"
#include "TimeUtil.h"
#include <functional>
#include <queue>
#include <vector>
#include <unordered_map>
#include <memory>
#include <random>
#include <Windows.h>
#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <condition_variable>
#include <malloc.h>
#include <time.h>
//typedef void(*KeyCB)(const Key& key);

using namespace HM;
using namespace std;
const int ThreadCount = 10;
const int ThreadCountWithMainThread = ThreadCount + 1;
const int ScreenGridPixelCount = 32;