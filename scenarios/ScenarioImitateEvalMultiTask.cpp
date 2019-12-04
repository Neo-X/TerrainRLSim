#include "ScenarioImitateEvalMultiTask.h"
#include "scenarios/ScenarioExpImitate.h"
#include "sim/CtTrackController.h"
#include "sim/CtPhaseController.h"
#include "sim/WaypointController.h"
#include "sim/SimCharSoftFall.h"
#include "util/FileUtil.h"
#include "sim/GroundFactory.h"
#include "util/MathUtil.h"
#include "sim/GroundVar2D.h"

const std::string gTerrainFilesKey = "TerrainFiles";

cScenarioImitateEvalMultiTask::cScenarioImitateEvalMultiTask() :
		cScenarioImitateEval()
{
}

cScenarioImitateEvalMultiTask::~cScenarioImitateEvalMultiTask()
{
}

void cScenarioImitateEvalMultiTask::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cScenarioImitateEval::ParseArgs(parser);


	ParseGroundParams(parser, groundParams);
}


void cScenarioImitateEvalMultiTask::Reset()
{
	cScenarioImitateEval::Reset();
	// int taskID = cMathUtil::RandInt(0, GetNumTasks()-1);
	// std::cout << "Setting new task: " << taskID << std::endl;
	// setTaskID(taskID);

}


void cScenarioImitateEvalMultiTask::ParseGroundParams(const std::shared_ptr<cArgParser>& parser, std::vector<cGround::tParams>& out_params)
{
	std::cout << "Parsing terrains file" << std::endl;
	std::string terrain_file = "";
	parser->ParseString("terrain_file", terrain_file);
	std::string fpath;
	bool succ = parser->ParseString("relative_file_path", fpath);
	if (succ)
	{
		terrain_file = fpath + terrain_file;
	}
	std::ifstream f_stream(terrain_file);
	Json::Reader reader;
	Json::Value root;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (terrain_file != "")
	{
		std::cout << "Loading terrains" << std::endl;
		// bool succ = cGroundFactory::ParseParamsJson(terrain_file, out_params);
		succ = this->LoadTerrains(root["TerrainFiles"], out_params);
		if (!succ)
		{
			printf("Failed to parse terrain params from %s\n", terrain_file.c_str());
			assert(false);
		}
	}
}

bool cScenarioImitateEvalMultiTask::LoadTerrains(const Json::Value& json, std::vector<cGround::tParams>& out_params)
{
	bool succ = true;
	bool is_array = json.isArray();
	succ &= is_array;

	if (is_array)
	{
		std::vector<std::string> files;
		int num_files = json.size();
		files.reserve(num_files);

		for (int i = 0; i < num_files; ++i)
		{
			Json::Value json_elem = json.get(i, 0);
			std::string curr_file = json_elem.asString();
			files.push_back(curr_file);
		}

		LoadTerrains(files, out_params);
	}

	return succ;
}

void cScenarioImitateEvalMultiTask::LoadTerrains(const std::vector<std::string>& motion_files, std::vector<cGround::tParams>& out_params)
{
	int num_files = static_cast<int>(motion_files.size());

	for (int f = 0; f < num_files; ++f)
	{
		const std::string& curr_file = motion_files[f];

		cMotion curr_motion;
		// bool succ = curr_motion.Load(_relativeFilePath + curr_file);
		bool succ = cGroundFactory::ParseParamsJson(curr_file, mGroundParams);

		if (succ)
		{
			std::cout << "loaded terrain file: " << curr_file << std::endl;
			out_params.push_back(mGroundParams);
		}
		else
		{
			assert(false);
		}
	}

}

void cScenarioImitateEvalMultiTask::setTaskID(size_t task)
{
	// std::cout << "This scenario does not support multiple tasks" << std::endl;
	_taskID = task;
	mGroundParams = groundParams[_taskID];
	mGround->SetParams(mGroundParams);
	mGround->SetParamBlend(mGroundParams.mBlend);
	auto ground_var2d = std::dynamic_pointer_cast<cGroundVar2D>(mGround);
	if ( ground_var2d != nullptr )
	{
		auto terrain_func = cTerrainGen2D::GetTerrainFunc(mGroundParams.mType);
		ground_var2d->SetTerrainFunc(terrain_func);
	}
}

size_t cScenarioImitateEvalMultiTask::getTaskID() const
{
	return _taskID;
}

size_t cScenarioImitateEvalMultiTask::GetNumTasks() const
{
	return groundParams.size();
}


