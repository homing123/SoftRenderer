#pragma once
#include "EngineUtil.h"

namespace HM
{
	class ThreadPool
	{
	public:
		ThreadPool()
		{
			Init();
		}

		~ThreadPool();

		void Init();
		size_t _WorkerCount;
		vector<thread> v_Workers;
		vector<bool> v_Working;

		queue<function<void()>> q_Jobs;

		condition_variable _CV;
		mutex _Mutex;

		void Work(int idx);

		void EnqueueJob(function<void()> job);
		void ChangeThreadCount(int workerCount);

		void WaitForJoin();

	private:
		bool _Stop;
	};
}