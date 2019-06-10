#include "GBuffer.h"
#include "opengl.h"

cGBuffer::cGBuffer(void)
{
}

bool cGBuffer::Init(unsigned int width, unsigned int height)
{
	bool succ = true;
	mWidth = width;
	mHeight = height;

	// normal
	glGenTextures(1, &mTextures[eGBufferTexNormal]);
	checkError();
	glBindTexture(GL_TEXTURE_2D, mTextures[eGBufferTexNormal]);
	checkError();

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	checkError();
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	checkError();
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	checkError();
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	checkError();

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, 0);
	checkError();
	glBindTexture(GL_TEXTURE_2D, 0);
	checkError();

	// depth buffer
	glGenTextures(1, &mTextures[eGBufferTexDepth]);
	checkError();
	glBindTexture(GL_TEXTURE_2D, mTextures[eGBufferTexDepth]);
	checkError();

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	checkError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	checkError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	checkError();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	checkError();
	float border_color[4] = { 1.f, 1.f, 1.f, 1.f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
	checkError();
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 
				0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	checkError();

	// linked depth stencil and render texture
	glGenFramebuffers(1, &mObject);
	checkError();
	glBindFramebuffer(GL_FRAMEBUFFER, mObject);
	checkError();
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
							GL_TEXTURE_2D, mTextures[eGBufferTexNormal], 0);
	checkError();
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 
							GL_TEXTURE_2D, mTextures[eGBufferTexDepth], 0);
	checkError();
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mTextures[ eGBufferTexDepth]);
	checkError();
	GLenum DrawBuffers[] = {GL_COLOR_ATTACHMENT0}; 
    glDrawBuffers(eGBufferTexMax - 1, DrawBuffers); // -1 for tex
    checkError();
	GLenum status;
	// GLenum z = GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER;
	if ((status = glCheckFramebufferStatus(GL_FRAMEBUFFER)) != GL_FRAMEBUFFER_COMPLETE) {
		std::printf("GBuffer: texture is incomplete\n");
		checkError();
		succ = false;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	checkError();
	mTexture = mTextures[eGBufferTexNormal];
	mDepthStencil = mTextures[eGBufferTexDepth];

    return succ;
}

void cGBuffer::BindTex(GLint tex_slot, eGBufferTex tex_type)
{
	glActiveTexture(tex_slot);
	checkError();
	glBindTexture(GL_TEXTURE_2D, mTextures[tex_type]);
	checkError();
}

void cGBuffer::Reshape(int w, int h)
{
	mWidth = w;
	mHeight = h;
	if (mObject != 0) // 0 indicates the device's frame buffer, so no need to resize it
	{
		for (int i = 0; i < eGBufferTexMax; ++i)
		{
			glDeleteTextures(1, &mTextures[i]);
			checkError();
		}
#ifdef USE_OpenGLES
		glDeleteFramebuffers(1, &mObject);
#else
		glDeleteFramebuffersEXT(1, &mObject);
#endif
		checkError();
		Init(w, h);
	}
}

cGBuffer::~cGBuffer(void)
{
}
