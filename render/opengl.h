/*
 *  opengl.h
 *
 *
 *  Created by Glen Berseth on 06/11/18.
 *
 */
#ifndef _opengl_h_
#define _opengl_h_

/// Helpful references
/// https://pandorawiki.org/Porting_to_GLES_from_GL
/// https://github.com/kmuzykov/custom-opengl-es-game-engine/blob/master/bullet-2.82-r2704/Extras/sph/common/glut.h

// #define USE_OpenGLES 1
#ifdef USE_OpenGLES
// #include <GLES2/gl2.h>  /* use OpenGL ES 2.x */
// #include <GLES2/gl2ext.h>  /* use OpenGL ES 2.x */
// #include <GLES3/gl3ext.h>  /* use OpenGL ES 2.x */
#include <GLES3/gl32.h>  /* use OpenGL ES 2.x */
#define EGL_EGLEXT_PROTOTYPES
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
/// Conflict with Eigen...
#undef Success
/// Conflict with google logging library
#undef Status
/// Load some GLUT defines

/* Mouse buttons. */
#define GLUT_LEFT_BUTTON                0
#define GLUT_MIDDLE_BUTTON              1
#define GLUT_RIGHT_BUTTON               2

/* Mouse button  state. */
#define GLUT_DOWN                       0
#define GLUT_UP 1
// #include <GL/glew.h>
#else
#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#endif
#include <iostream>

// namespace Util {

static const char*
ErrorString( GLenum error )
{
    const char*  msg="";
    switch( error ) {
#define Case( Token )  case Token: msg = #Token; break;
            Case( GL_NO_ERROR );
            Case( GL_INVALID_VALUE );
            Case( GL_INVALID_ENUM );
            Case( GL_INVALID_OPERATION );
#ifndef USE_OpenGLES
            Case( GL_STACK_OVERFLOW );
            Case( GL_STACK_UNDERFLOW );
#endif
            Case( GL_OUT_OF_MEMORY );
#undef Case
    }

    return msg;
}

//----------------------------------------------------------------------------

static void _CheckError(const char* file, int line)
{
    GLenum  error;
    // std::cout << "checking gl error: " << std::endl;
#ifdef USE_OpenGLES
    while ((error = eglGetError()) != EGL_SUCCESS )
#else
	while ((error = glGetError()) != GL_NO_ERROR )
#endif
    {
        if( error == 0 )
        {
            std::cerr << "glGetError failed!\n" ;
            continue ;

        }
        std::cout <<  "File " << file << " line " << line <<": OpenGL Error: " << error << " msg:" << ErrorString(error) << std::endl;
    }

}

#define checkError()  _CheckError( __FILE__, __LINE__ )


#endif
