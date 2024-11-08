#include "ThreadPool.h"

void ThreadPool::Init()
{
	_Stop = false;
	_WorkerCount = ThreadCount;
	for (size_t i = 0; i < _WorkerCount; i++)
	{
		v_Working.push_back(false);
		v_Workers.push_back(thread(&ThreadPool::Work,this, i));
	}
}
ThreadPool::~ThreadPool()
{
	_Stop = true;
	_CV.notify_all();
	for (auto& w : v_Workers)
	{
		w.join();
	}
}
void ThreadPool::Work(int idx)
{
	while (true)
	{

		unique_lock<mutex> lock(_Mutex);
		v_Working[idx] = false;

		_CV.wait(lock, [this]()->bool { return !q_Jobs.empty() || _Stop; });	//true 면 넘어가고 아니면 대기상태로 전환 (대기상태 = 깨워줘야함)
		//맨 앞의 job 가져오기
		function<void()> job;
		if (q_Jobs.empty() == false) 
		{
			v_Working[idx] = true;
			job = move(q_Jobs.front());
			q_Jobs.pop();
			lock.unlock();
		}
		else
		{
			lock.unlock();
		}

		if (job != nullptr) 
		{
			job();
		}
		if (_Stop)
		{
			return;
		}
	}
}

void ThreadPool::EnqueueJob(function<void()> job)
{
	{
		lock_guard<mutex> lock(_Mutex);
		q_Jobs.push(move(job));
	}
	_CV.notify_one();
}
void ThreadPool::ChangeThreadCount(int workerCount)
{
	if (workerCount > _WorkerCount)
	{
		for (size_t i = _WorkerCount; i < workerCount; i++)
		{
			v_Working.push_back(false);
			v_Workers.push_back(thread(&ThreadPool::Work, this, i));
		}
		_WorkerCount = workerCount;
	}
	else if (_WorkerCount > workerCount)
	{
		//전체 종료 및 대기
		_Stop = true;
		_CV.notify_all();
		for (auto& w : v_Workers)
		{
			w.join();
		}
		_Stop = false;

		//갯수 줄이기
		for (size_t i = 0; i < _WorkerCount - workerCount; i++)
		{
			v_Working.pop_back();
			v_Workers.pop_back();
		}
		_WorkerCount = workerCount;

		//다시 가동
		for (size_t i = 0; i < _WorkerCount; i++)
		{
			v_Workers[i] = thread(&ThreadPool::Work, this, i);
		}

	}
}

void ThreadPool::WaitForJoin()
{
	bool isJoin;

	while (true)
	{
		if (q_Jobs.empty())
		{
			break;
		}
	}

	while (true)
	{
		isJoin = true;
		for (int i = 0; i < _WorkerCount; i++)
		{
			if (v_Working[i] == true)
			{
				isJoin = false;
			}			
		}
		

		if (isJoin == true)
		{
			break;
		}
	}
}