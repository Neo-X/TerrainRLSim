#include "sim/GroundDynamicCharacters3D.h"
#include "sim/TerrainGen3D.h"

const double gDefaultHeight = 0;

cGroundDynamicCharacters3D::cGroundDynamicCharacters3D()
{
}

cGroundDynamicCharacters3D::~cGroundDynamicCharacters3D()
{
}

void cGroundDynamicCharacters3D::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	cGroundPlane::Init(world, params);
	// BuildObstacles();
}

cGroundDynamicCharacters3D::eClass cGroundDynamicCharacters3D::GetGroundClass() const
{
	return eClassDynamicCharacters3D;
}

int cGroundDynamicCharacters3D::GetBlendParamSize() const
{
	return static_cast<int>(cTerrainGen3D::eParamsMax);
}

void cGroundDynamicCharacters3D::SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters)
{
	mChars = characters;
}

void cGroundDynamicCharacters3D::SetChar(const std::shared_ptr<cSimCharacter>& character)
{
	mChar = character;
}

void cGroundDynamicCharacters3D::SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel,
													bool& out_valid_sample) const
{
	out_h = gDefaultHeight;
	out_vel = tVector::Zero();

	bool foundIntersectedChar = false;

	// Check to see if this sample point contains any of the other characters
	for(std::vector<std::shared_ptr<cSimCharacter>>::const_iterator it = mChars.begin(); it != mChars.end(); ++it) 
	{
		tVector aabb_min;
		tVector aabb_max;
		(*it)->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = (*it)->CalcCOMVel(); // Works for new LLC...
			// out_vel = (*it)->GetRootVel(); // Works for old LLC
			foundIntersectedChar = true;
			break;
		}
	}

	// If we havent found one intersection amongst the additional character, then check the default one as well
	if (!foundIntersectedChar)
	{
		tVector aabb_min;
		tVector aabb_max;
		mChar->CalcAABB(aabb_min, aabb_max);
		
		if (cMathUtil::ContainsAABBXZ(pos, aabb_min, aabb_max))
		{
			out_h = aabb_max[1];
			out_vel = mChar->CalcCOMVel();
			// out_vel = mChar->GetRootVel(); // Works for old LLC
		}
	}

	out_h = SampleHeight(pos, out_valid_sample);
	out_valid_sample = true;
}
