#pragma once

#include "sim/GroundPlane.h"
#include "sim/SimCharacter.h"

class cGroundDynamicCharacters3D : public cGroundPlane
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGroundDynamicCharacters3D();
	virtual ~cGroundDynamicCharacters3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);

	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;

	virtual void SetChars(const std::vector<std::shared_ptr<cSimCharacter>>& characters);
	virtual void SetChar(const std::shared_ptr<cSimCharacter>& character);
	virtual const std::vector<std::shared_ptr<cSimCharacter>> GetChars() const;
	virtual eClass GetGroundClass() const;

protected:
	// This is the vector of 'other characters'
	std::vector<std::shared_ptr<cSimCharacter>> mChars;

	// This is the main character
	std::shared_ptr<cSimCharacter> mChar;

	virtual int GetBlendParamSize() const;
};
