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

		_CV.wait(lock, [this]()->bool { return !q_Jobs.empty() || _Stop; });	//true �� �Ѿ�� �ƴϸ� �����·� ��ȯ (������ = ���������)
		//�� ���� job ��������
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
		//��ü ���� �� ���
		_Stop = true;
		_CV.notify_all();
		for (auto& w : v_Workers)
		{
			w.join();
		}
		_Stop = false;

		//���� ���̱�
		for (size_t i = 0; i < _WorkerCount - workerCount; i++)
		{
			v_Working.pop_back();
			v_Workers.pop_back();
		}
		_WorkerCount = workerCount;

		//�ٽ� ����
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