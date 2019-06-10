#pragma once

#include <stdlib.h>
#include "opengl.h"
#include <fstream>

#include "util/MathUtil.h"

class cShader
{
public:
	/// Stuff to prevent the auto creation of copy constructors
	cShader(void);
	// cShader(const cShader&) = delete;
	// cShader& operator=(const cShader&) = delete;
	~cShader(void);

	virtual bool BuildShader(const std::string& vs_filename, const std::string& ps_filename);
	virtual void BindShaderAttribute(GLuint& attribute_handle, const std::string& attribute_name);
	virtual void BindShaderUniform(GLuint& uniform_handle, const std::string& uniform_name);

	virtual void Bind() const;
	virtual void Unbind() const;

	virtual void SetUniform3(GLuint handle, const tVector& data) const;
	virtual void SetUniform4(GLuint handle, const tVector& data) const;
	virtual void SetUniform2(GLuint handle, const tVector& data) const;
	virtual void setUniformMat3(GLuint handle, const tMatrix3& data) const;
	virtual void setUniformMat4(GLuint handle, const tMatrix& data) const;
	virtual void setUniformMat3(GLuint handle, const tMatrix3f& data) const;
	virtual void setUniformMat4(GLuint handle, const tMatrixf& data) const;

	virtual GLuint GetProg() const;
	virtual void setShaderRelativePath(std::string path);

protected:
	GLuint mProg;
	std::string mRelativePath;

	virtual GLuint LoadShader(const std::string& filename, GLenum shader_type);
	virtual GLuint CreateShader(GLuint& vsh_handle, GLuint& psh_handle);
};
