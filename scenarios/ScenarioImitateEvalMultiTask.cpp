#include "ScenarioImitateEvalMultiTask.h"
#include "scenarios/ScenarioExpImitate.h"
#include "sim/CtTrackController.h"
#include "sim/CtPhaseController.h"
#include "sim/WaypointController.h"
#include "sim/SimCharSoftFall.h"
#include "util/FileUtil.h"
#include "sim/GroundFactory.h"

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

	ParseGroundParams(parser, mGroundParams);

}

void cScenarioImitateEvalMultiTask::ParseGroundParams(const std::shared_ptr<cArgParser>& parser, cGround::tParams& out_params) const
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

bool cScenarioImitateEvalMultiTask::LoadTerrains(const Json::Value& json, cGround::tParams& out_params) const
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

void cScenarioImitateEvalMultiTask::LoadTerrains(const std::vector<std::string>& motion_files, cGround::tParams& out_params) const
{
	int num_files = static_cast<int>(motion_files.size());

	for (int f = 0; f < num_files; ++f)
	{
		const std::string& curr_file = motion_files[f];

		cMotion curr_motion;
		// bool succ = curr_motion.Load(_relativeFilePath + curr_file);
		bool succ = cGroundFactory::ParseParamsJson(curr_file, out_params);
		if (succ)
		{
			std::cout << "loaded terrain file: " << curr_file << std::endl;
		}
		else
		{
			assert(false);
		}
	}

}
