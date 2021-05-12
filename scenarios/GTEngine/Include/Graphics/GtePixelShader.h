// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2016
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 2.1.1 (2016/02/04)

#pragma once

#include <Graphics/GteShader.h>

namespace gte
{

class GTE_IMPEXP PixelShader : public Shader
{
public:
    // Construction.
#if defined(GTE_DEV_OPENGL)
    PixelShader(GLSLReflection const& reflector);
#else
    PixelShader(HLSLShader const& program);
#endif
};

}
