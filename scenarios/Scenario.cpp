#include "Scenario.h"
#include <iostream>

cScenario::cScenario()
{
	mResetCallback = nullptr;
	_relativeFilePath = "";
	_taskID=0;
}

void cScenario::Init()
{
}

void cScenario::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{

}

void cScenario::Reset()
{
	if (mResetCallback != nullptr)
	{
		mResetCallback();
	}
}

void cScenario::Clear()
{
}

void cScenario::setTaskID(size_t task)
{
	std::cout << "This scenario does not support multiple tasks" << std::endl;
}

size_t cScenario::getTaskID() const
{
	return _taskID;
}

size_t cScenario::GetNumTasks() const
{
	return 1;
}

void cScenario::Run()
{
}

void cScenario::Shutdown()
{
}

bool cScenario::IsDone() const
{
	return false;
}

void cScenario::Update(double time_elapsed)
{
}

void cScenario::SetResetCallback(tCallbackFunc func)
{
	mResetCallback = func;
}

std::string cScenario::GetName() const
{
	return "No Name";
}

cScenario::~cScenario()
{
}

void cScenario::setRelativeFilePath(std::string path)
{
	_relativeFilePath = path;
}

std::string cScenario::getRelativeFilePath()
{
	return _relativeFilePath;
}
