#include "DrawScenarioImitateVizEval.h"
#include "scenarios/ScenarioImitateVizEval.h"
#include "anim/KinCharacter.h"
#include "anim/KinSimCharacter.h"
#include "render/DrawUtil.h"
#include "render/DrawCharacter.h"

const double gLinkWidth = 0.025;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(1, 1, 1, 1);
const tVector gKinCharOffset = tVector(0, 0, 0.5, 0);

cDrawScenarioImitateVizEval::cDrawScenarioImitateVizEval(cCamera& cam)
	: cDrawScenarioPoliEval(cam)
{
	mDrawKinChar = true;
	mTrackKinChar = false;
	zoom = 0.0;
}

cDrawScenarioImitateVizEval::~cDrawScenarioImitateVizEval()
{
}

void cDrawScenarioImitateVizEval::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawScenarioPoliEval::ParseArgs(parser);
	parser->ParseDouble("camera_zoom", this->zoom);
}

void cDrawScenarioImitateVizEval::Init()
{
	cDrawScenarioPoliEval::Init();
	mDrawKinChar = true;
	ToggleCamTrackMode(eCamTrackModeXZ);
	tVector focus = mCam.GetFocus();
	tVector cam_offset = -(mCam.GetFocus() - mCam.GetPosition());
	double w = mCam.GetWidth();
	double h = mCam.GetHeight();

	double delta_scale = 1 - zoom;
	tVector delta = cam_offset * delta_scale;
	tVector camPosition = focus + delta;
	w *= delta_scale;
	h *= delta_scale;
	mCam.Resize(w, h);
	mCam.SetFocus(focus);
	mCam.SetPosition(camPosition);
}

void cDrawScenarioImitateVizEval::Reset()
{
	auto eval_scene = std::dynamic_pointer_cast<cScenarioPoliEval>(mScene);
	if (eval_scene != nullptr)
	{
		eval_scene->EndEpisodeRecord();
	}
	cDrawScenarioPoliEval::Reset();
}

void cDrawScenarioImitateVizEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioPoliEval::Keyboard(key, x, y);
	switch (key)
	{
	case 'k':
		ToggleDrawKinChar();
		break;
	case '8':
		ToggleTrackKinCharacter();
		break;
	default:
		break;
	}
}

void cDrawScenarioImitateVizEval::BuildScene(std::shared_ptr<cScenarioSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioImitateVizEval>(new cScenarioImitateVizEval());
}

void cDrawScenarioImitateVizEval::DrawCharacters() const
{
	cDrawScenarioPoliEval::DrawCharacters();

	if (mDrawKinChar)
	{
		DrawKinChar();
	}
}

void cDrawScenarioImitateVizEval::DrawKinChar() const
{
	const auto& kin_char = GetKinChar();
	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(GetDrawKinCharOffset());
	cDrawCharacter::Draw(*kin_char.get(), gLinkWidth, gFilLColor, GetLineColor());
	const std::shared_ptr<cKinSimCharacter> kin_char_tmp = std::dynamic_pointer_cast<cKinSimCharacter >(kin_char);
	if ( kin_char_tmp != nullptr)
	{
		Eigen::MatrixXd out_draw_shapes = kin_char_tmp->GetDrawShapeDefs();
		// kin_char_tmp->getDrawShapes(out_draw_shapes);
		// cDrawCharacter::DrawShapes(kin_char_tmp, out_draw_shapes, gFilLColor, GetLineColor());
		cDrawCharacter::DrawShapes(*kin_char_tmp, gFilLColor, GetLineColor());
		/// try to get poli states
		Eigen::VectorXd out_pose;
		kin_char_tmp->BuildKinCharPoliStatePose(out_pose);
		Eigen::VectorXd out_pose_vel;
		kin_char_tmp->BuildKinCharPoliStateVel(out_pose_vel);

	}
	cDrawUtil::PopMatrix();
}

const std::shared_ptr<cKinCharacter>& cDrawScenarioImitateVizEval::GetKinChar() const
{
	auto scene = std::dynamic_pointer_cast<cScenarioImitateVizEval>(mScene);
	return scene->GetKinChar();
}

void cDrawScenarioImitateVizEval::ToggleDrawKinChar()
{
	mDrawKinChar = !mDrawKinChar;
}

tVector cDrawScenarioImitateVizEval::GetDrawKinCharOffset() const
{
	/*
	const auto& kin_char = GetKinChar();
	const auto& sim_char = mScene->GetCharacter();
	tVector kin_pos = kin_char->GetRootPos();
	tVector sim_pos = sim_char->GetRootPos();
	tVector target_pos = sim_pos += tVector(-1, 0, 1, 0);
	target_pos[1] = kin_pos[1];
	return target_pos - kin_pos;
	*/
	eDrawMode draw_mode = GetDrawMode();
	if (draw_mode == eDrawMode2D)
	{
		return gKinCharOffset;
	}
	return tVector::Zero();
}

void cDrawScenarioImitateVizEval::ToggleTrackKinCharacter()
{
	mTrackKinChar = !mTrackKinChar;
	// std::cout << "toggle track kin character." << std::endl;
}

tVector cDrawScenarioImitateVizEval::GetCamTrackPos() const
{
	if (mTrackKinChar)
	{
		return GetKinChar()->CalcCOM();
	}
	else
	{
		return cDrawScenarioPoliEval::GetCamTrackPos();
	}
}

void cDrawScenarioImitateVizEval::DrawMisc() const
{
	cDrawScenarioPoliEval::DrawMisc();

	// DrawTargetPos();
}

void cDrawScenarioImitateVizEval::DrawTargetPos() const
{
	/// Draw an invisible sphere so that the ground is rendered at the origin..
	const double r = 0.1;
	const double line_h = 10;
	const double line_w = 3;
	const tVector col = tVector(1, 0, 0, 0.00);

	// auto scene = std::dynamic_pointer_cast<cScenarioExpImitateTarget>(mScene);
	const tVector& target_pos = tVector(0,0,0,0);
	double reset_dist = 0.5;

	cDrawUtil::PushMatrix();

	cDrawUtil::Translate(target_pos);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawSphere(r);

	// cDrawUtil::Translate(tVector(0, 0.1, 0, 0));
	// cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);

	cDrawUtil::PopMatrix();

	cDrawUtil::SetLineWidth(line_w);
	cDrawUtil::DrawLine(target_pos, target_pos + tVector(0, line_h, 0, 0));
	cDrawUtil::SetLineWidth(0.5 * line_w);

	cDrawUtil::PushMatrix();
	cDrawUtil::Translate(target_pos);
	// cDrawUtil::Rotate(0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawDisk(reset_dist, cDrawUtil::eDrawWire);
	cDrawUtil::PopMatrix();
}

void cDrawScenarioImitateVizEval::Draw()
{
	// std::cout << "Updating camera location:" << std::endl;
	// std::cout << "view mode: " << mCamTrackMode << std::endl;
	// UpdateCamera();
	/*
	tVector cam_focus = mCam.GetFocus();
	tVector target_focus = GetCamTrackPos();
	cam_focus[0] = target_focus[0];
	cam_focus[2] = target_focus[2];
	mCam.TranslatePos(cam_focus);
	*/
	cDrawScenarioPoliEval::Draw();
	// DrawTargetPos();
	// DrawGround();
}
