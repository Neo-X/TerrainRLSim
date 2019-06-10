#include "TextureUtil.h"
#include <iostream>

void CreateFrameBuffer( GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil, 
					   int width, int height, int depth, int channels, GLenum format, 
					   GLenum type, bool mipmaps )
{
	bool is_3d_tex = depth > 1;
	glGenTextures( 1, &texture );
	checkError();
	GLenum tex_type = GL_TEXTURE_2D;
	if (is_3d_tex)
	{
		tex_type = GL_TEXTURE_3D;
	}

	glBindTexture( tex_type, texture );
	checkError();
	
	glTexParameteri(tex_type, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	checkError();
	glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	checkError();
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	checkError();
	glTexParameteri(tex_type, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	checkError();
	
	if (is_3d_tex)
	{
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		checkError();
	}

	// generate the top level
	if (is_3d_tex)
	{
		glTexImage3D(GL_TEXTURE_3D, 0, channels, width, height, depth, 0, format, type, 0);
		checkError();
	}
	else
	{
		glTexImage2D(GL_TEXTURE_2D, 0, channels, width, height, 0, format, type, 0);
		checkError();
	}

	// generate the mips
	if ( mipmaps )
	{
		glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		checkError();
		glEnable(tex_type);
		checkError();
		glGenerateMipmap(tex_type);
		checkError();
	}

	glBindTexture(tex_type, 0);
	checkError();
	
	// depth buffer
	glGenRenderbuffers(1, &depth_stencil);
	checkError();
	glBindRenderbuffer(GL_RENDERBUFFER, depth_stencil);
	checkError();
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height );
	checkError();
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	checkError();

	// linked depth stencil and render texture
	glGenFramebuffers(1, &buffer_obj);
	checkError();
	if (is_3d_tex)
	{
		glBindFramebuffer(GL_RENDERBUFFER, buffer_obj);
		checkError();
		glFramebufferTexture(GL_RENDERBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);
		checkError();
	}
	else
	{
		glBindFramebuffer(GL_FRAMEBUFFER, buffer_obj);
		checkError();
#ifdef _LINUX_
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
		checkError();
#else
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);
		checkError();
#endif
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_stencil );
		checkError();
	}

	GLenum status;
	// GLenum z = GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER;
	if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
		checkError();
		if (is_3d_tex)
		{
			std::cout <<"fail to create 3d texture" << std::endl;
		}
		else
		{
			std::cout << "TextureUtil: texture is incomplete" << std::endl;
		}
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	checkError();
}

void DeleteFrameBuffer( GLuint& buffer_obj, GLuint& texture, GLuint& depth_stencil )
{
#ifdef USE_OpenGLES
	glDeleteRenderbuffers(1, &depth_stencil);
	checkError();
	glDeleteTextures(1, &texture);
	checkError();
	glDeleteFramebuffers(1, &buffer_obj);
	checkError();
#else
	glDeleteRenderbuffersEXT(1, &depth_stencil);
	checkError();
	glDeleteTextures(1, &texture);
	checkError();
	glDeleteFramebuffersEXT(1, &buffer_obj);
	checkError();
#endif
}

tVector ReadTexel(int x, int y, int w, int h, const std::vector<GLfloat>& data)
{
	const int num_channels = 4;
	size_t idx = static_cast<size_t>((w * y + x) * num_channels);
	GLfloat r = data[idx];
	GLfloat g = data[idx + 1];
	GLfloat b = data[idx + 2];
	GLfloat a = data[idx + 3];

	tVector texel = tVector(r, g, b, a);
	return texel;
}
