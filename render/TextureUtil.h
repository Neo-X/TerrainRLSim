#pragma once

#include <stdlib.h>
#include "opengl.h"
#include <fstream>

#include "util/MathUtil.h"

void CreateFrameBuffer( GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil, 
					   int width, int height, int depth, int channels, GLenum format, GLenum type, bool mipmaps = false );
void ReshapeFrameBuffer( GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil, 
					   int width, int height );
void DeleteFrameBuffer( GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil );
tVector ReadTexel(int x, int y, int w, int h, const std::vector<GLfloat>& data);
