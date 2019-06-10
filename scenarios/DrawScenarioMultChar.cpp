#include "DrawScenarioMultChar.h"
#include "DrawScenarioImitateStepEval.h"
#include "scenarios/ScenarioMultChar.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawUtil.h"
#include "sim/WaypointController.h"
#include "sim/BaseControllerMACE.h"
#include "sim/BaseControllerMACEDPG.h"

const tVector gFillTint = tVector(1, 1, 1, 1);

cDrawScenarioMultChar::cDrawScenarioMultChar(cCamera& cam)
	: cDrawScenarioHikeEval(cam)
{
	mDrawOtherChar = true;
}

cDrawScenarioMultChar::~cDrawScenarioMultChar()
{

}

void cDrawScenarioMultChar::ResetCallback()
{
	cDrawScenarioImitateTargetEval::ResetCallback();

	auto scene = std::dynamic_pointer_cast<cScenarioMultChar>(mScene);
	scene->EnableRandTargetPos(true);
}

void cDrawScenarioMultChar::DrawStepPlan() const
{
	cDrawScenarioImitateStepEval::DrawStepPlan();

	const tVector& pos0_col = tVector(0, 1, 0, 0.5);
	const tVector& pos1_col = tVector(0, 0.5, 0, 0.5);
	const tVector& heading_col = tVector(1, 0, 0, 0.5);

	// auto step_scene = std::dynamic_pointer_cast<cScenarioExpImitateStep>(mScene);

	auto scene = std::dynamic_pointer_cast<cScenarioMultChar>(mScene);
	for (size_t a =0; a < scene->mChars.size(); a++)
	// for(std::vector<std::shared_ptr<cSimCharacter>>::iterator it = scene->mChars.begin(); it != scene->mChars.end(); ++it)
	{
		cDrawUtil::PushMatrix();
		//cDrawUtil::Translate(tVector(0, 0, 0.5, 0));
		std::shared_ptr<cSimCharacter> character = scene->mChars[a];
		// cDrawSimCharacter::Draw(*(character.get()), gFillTint, GetLineColor(), enable_draw_shape);
		cDrawUtil::PopMatrix();

		// std::shared_ptr<cCharController> ctrl = character->GetController();
		std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(character->GetController());
		// std::shared_ptr<cBipedStepController3D > ctrl_ = std::static_pointer_cast<cBipedStepController3D >(character->GetController());
		auto ctrl_ = std::dynamic_pointer_cast<cWaypointController>(character->GetController());
		// std::dynamic_pointer_cast<cWaypointController>
		if ( ctrl_ != nullptr )
		{
			const auto& step_plan = ctrl_->GetStepPlan();

			DrawStepPos(step_plan.mStepPos0, pos0_col);
			DrawStepPos(step_plan.mStepPos1, pos1_col);
			DrawRootHeading(step_plan, heading_col);
		}
		else
		{
			std::cout << "Controller type is not a biped step" << std::endl;
		}
	}

}

/*
std::string cDrawScenarioMultChar::BuildTextInfoStr() const
{
	const auto& character = mScene->GetCharacter();
	tVector com = character->CalcCOM();
	tVector com_vel = character->CalcCOMVel();
	char buffer[128];

	std::string info_str = cDrawScenarioSimChar::BuildTextInfoStr();

	auto poli_eval = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	int num_samples = poli_eval->GetNumEpisodes();
	//double avg_dist = poli_eval->GetAvgDist();
	//sprintf(buffer, "Avg Dist: %.2f (%i)\n", avg_dist, num_samples);
	//info_str += buffer;

	double total_reward = poli_eval->GetCurrCumulativeReward();
	sprintf(buffer, "Total Reward: %.2f\n", total_reward);
	info_str += buffer;

	double avg_reward = poli_eval->CalcAvgReward();
	sprintf(buffer, "Avg Reward: %.2f (%i)\n", avg_reward, num_samples);
	info_str += buffer;

	return info_str;
}
*/

void cDrawScenarioMultChar::SetTargetPos(const tVector& pos)
{
	/*
	auto scene = std::dynamic_pointer_cast<cScenarioImitateStepEval>(mScene);
	scene->SetTargetPos(pos);
	scene->EnableTargetPos(true);
	scene->EnableRandTargetPos(false);
	*/
}

void cDrawScenarioMultChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioHikeEval::Keyboard(key, x, y);

	switch (key)
	{
	case 'u':
		mDrawOtherChar = !mDrawOtherChar;
		break;
	default:
		break;
	}
}

void cDrawScenarioMultChar::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioMultChar>(new cScenarioMultChar());
}

void cDrawScenarioMultChar::DrawCharacters() const
{
	cDrawScenarioHikeEval::DrawCharacters();

	// DrawTarget( mScene->GetCharacter() );
	const double marker_size = 0.05;
	const double vel_scale = 0.025;
	const tVector pos_col = tVector(1, 0, 0, 0.5);
	const tVector vel_col = tVector(0, 0.75, 0, 0.5);
	const tVector terrain_col = tVector(0, 0, 1, 0.5);
	const auto& character = mScene->GetCharacter();
	const auto& ground = mScene->GetGround();
	

	if (mDrawOtherChar)
	{
		auto scene = std::dynamic_pointer_cast<cScenarioMultChar>(mScene);
		for(std::vector<std::shared_ptr<cSimCharacter>>::iterator it = scene->mChars.begin(); it != scene->mChars.end(); ++it) 
		{
			cDrawUtil::PushMatrix();
			//cDrawUtil::Translate(tVector(0, 0, 0.5, 0));
			const auto& character = *it;
			bool enable_draw_shape = mEnableCharDrawShapes;
			cDrawSimCharacter::Draw(*(character.get()), gFillTint, GetLineColor(), enable_draw_shape);
			cDrawUtil::PopMatrix();

			if ( mDrawFeatures )
			{
				cDrawSimCharacter::DrawCharFeatures(*(character.get()), *ground.get(),
					marker_size, vel_scale, pos_col, vel_col, GetVisOffset());
				cDrawSimCharacter::DrawTerainFeatures(*(character.get()), marker_size, terrain_col, GetVisOffset());
			}

			// Draw target stuff
			DrawTarget( (*it) );
		}
	}
	DrawTarget( character );
}

void cDrawScenarioMultChar::DrawTarget(const std::shared_ptr<cSimCharacter>& character) const
{
	/*
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(character->GetCurrentGroundTarget());
	cDrawUtil::DrawSphere( 0.25 );
	cDrawUtil::PopMatrix();

	cDrawUtil::DrawLine( groundProjectedPosition, character->GetCurrentGroundTarget() );
	*/
	tVector groundProjectedPosition = tVector(character->GetRootPos()[0], 0, character->GetRootPos()[2], 0);
	const double r = 0.1;
	const double line_h = 2;
	const double line_w = 2;
	const tVector col = tVector(1, 0, 0, 0.5);

	auto scene = std::dynamic_pointer_cast<cScenarioExpImitateTarget>(mScene);
	const tVector& target_pos = character->GetCurrentGroundTarget();
	double reset_dist = scene->GetTargetResetDist();

	cDrawUtil::PushMatrix();

	cDrawUtil::Translate(target_pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);

	cDrawUtil::Translate(tVector(0, 0.1, 0, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);

	cDrawUtil::PopMatrix();

	cDrawUtil::SetLineWidth(line_w);
	cDrawUtil::DrawLine(target_pos, target_pos + tVector(0, line_h, 0, 0));
	cDrawUtil::SetLineWidth(0.5 * line_w);

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(target_pos + tVector(0, 0.1, 0, 0));
	cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);
	cDrawUtil::PopMatrix();
	// cDrawUtil::SetColor(tVector(0.2, 1.0, 0.2, 0.5));
	cDrawUtil::DrawLine( groundProjectedPosition, character->GetCurrentGroundTarget() );
}

void cDrawScenarioMultChar::UpdateTracer(double time_elapsed)
{
	cDrawScenarioHikeEval::UpdateTracer(time_elapsed);

	auto ctrl = mScene->GetCharacter()->GetController();

	const std::shared_ptr<cBaseControllerMACE> mace_ctrl = std::dynamic_pointer_cast<cBaseControllerMACE>(ctrl);
	const std::shared_ptr<cBaseControllerMACEDPG> mace_dpg_ctrl = std::dynamic_pointer_cast<cBaseControllerMACEDPG>(ctrl);

	bool is_mace_ctrl = (mace_ctrl != nullptr) || (mace_dpg_ctrl != nullptr);
	if (is_mace_ctrl)
	{
		// std::cout << "balh" << std::endl;
		int action_id = ctrl->GetCurrActionID();
		int trace_handle = mTraceHandles[0];
		mTracer.SetTraceColIdx(trace_handle, action_id);
	}

	mTracer.Update(time_elapsed);

}

void cDrawScenarioMultChar::InitTracer()
{
	cDrawScenarioHikeEval::InitTracer();
	// mTraceHandles.clear();
	// mTracer.Init(gTracerBufferSize, gTracerSamplePeriod);
	auto scene = std::dynamic_pointer_cast<cScenarioMultChar>(mScene);
	for (size_t i = 0; i < scene->mChars.size(); i++)
	{
		tVectorArr tracer_cols;
		tracer_cols.push_back(tVector(0, 0, 1, 0.5));
		tracer_cols.push_back(tVector(1, 0, 0, 0.5));
		tracer_cols.push_back(tVector(0, 0.5, 0, 0.5));
		tracer_cols.push_back(tVector(0.75, 0, 0.75, 0.5));
		tracer_cols.push_back(tVector(0, 0.5, 0.5, 0.5));
		tracer_cols.push_back(tVector(0, 0, 0, 0.5));
		int handle = AddCharTrace(scene->mChars[i], tracer_cols);
		mTraceHandles.push_back(handle);
	}
}
