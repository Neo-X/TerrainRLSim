#pragma once

#include "anim/Character.h"
#include "anim/KinSimCharacter.h"

class cDrawCharacter
{
public:
	static void Draw(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& line_col);
	static void DrawShape(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
	static void DrawHeading(const cCharacter& character, double arrow_size, const tVector& arrow_col, const tVector& offset);
	static void DrawShapes(const cKinSimCharacter& character, const tVector& fill_tint, const tVector& line_col);

protected:
	static void DrawShapeBox(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeCapsule(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeSphere(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeCylinder(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
};
