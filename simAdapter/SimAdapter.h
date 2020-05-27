/*
 * SimAdapter.h
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#ifndef SIMADAPTER_H_
#define SIMADAPTER_H_
#include <vector>
#include <string>
#include "scenarios/Scenario.h"
#include "scenarios/ScenarioSimChar.h"
#include "render/opengl.h"
#ifdef USE_OpenGLES
#include <GLES3/gl3.h>  /* use OpenGL ES 3.x */
#include <EGL/egl.h>
#define EGL_EGLEXT_PROTOTYPES
#include <EGL/eglext.h>
#else
#include <GL/glx.h>
#include <X11/X.h>
#include <X11/Xlib.h>
#endif
#include <sys/stat.h>

/*
 * Return true if the file exists
 */
inline bool file_exists_test (const std::string& name) {
	std::cout << "Checking if file exsits: " << name << std::endl;
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

static void
make_z_rot_matrix(GLfloat angle, GLfloat *m)
{
   float c = cos(angle * M_PI / 180.0);
   float s = sin(angle * M_PI / 180.0);
   int i;
   for (i = 0; i < 16; i++)
      m[i] = 0.0;
   m[0] = m[5] = m[10] = m[15] = 1.0;

   m[0] = c;
   m[1] = s;
   m[4] = -s;
   m[5] = c;
}

static void
make_scale_matrix(GLfloat xs, GLfloat ys, GLfloat zs, GLfloat *m)
{
   int i;
   for (i = 0; i < 16; i++)
      m[i] = 0.0;
   m[0] = xs;
   m[5] = ys;
   m[10] = zs;
   m[15] = 1.0;
}

/*
 * row major matrix? I guess not....
 */
static void
make_translation_matrix(GLfloat x, GLfloat y, GLfloat z, GLfloat *m)
{
   int i;
   for (i = 0; i < 16; i++)
      m[i] = 0.0;

   m[0] = m[5] = m[10] = m[15] = 1.0;
	// m[3] = x;
	// m[7] = y;
	// m[11] = z;
	m[12] = x;
	m[13] = y;
	m[14] = z;
}

/*
 * row major matrix? I guess not....
 */
static void
make_identity_matrix(GLfloat *m)
{
   int i;
   for (i = 0; i < 16; i++)
      m[i] = 0.0;

   m[0] = m[5] = m[10] = m[15] = 1.0;
}


static void
mul_matrix(GLfloat *prod, const GLfloat *a, const GLfloat *b)
{
#define A(row,col)  a[(col<<2)+row]
#define B(row,col)  b[(col<<2)+row]
#define P(row,col)  p[(col<<2)+row]
   GLfloat p[16];
   GLint i;
   for (i = 0; i < 4; i++) {
      const GLfloat ai0=A(i,0),  ai1=A(i,1),  ai2=A(i,2),  ai3=A(i,3);
      P(i,0) = ai0 * B(0,0) + ai1 * B(1,0) + ai2 * B(2,0) + ai3 * B(3,0);
      P(i,1) = ai0 * B(0,1) + ai1 * B(1,1) + ai2 * B(2,1) + ai3 * B(3,1);
      P(i,2) = ai0 * B(0,2) + ai1 * B(1,2) + ai2 * B(2,2) + ai3 * B(3,2);
      P(i,3) = ai0 * B(0,3) + ai1 * B(1,3) + ai2 * B(2,3) + ai3 * B(3,3);
   }
   memcpy(prod, p, sizeof(p));
#undef A
#undef B
#undef PROD
}


#ifdef USE_OpenGLES
#include <exception>

class EGLException:public std::exception
{
  public:
    const char* message;
    EGLException(const char* mmessage): message(mmessage) {}

    virtual const char* what() const throw()
    {
      return this->message;
    }
};

class EGLReturnException: private EGLException
{
  using EGLException::EGLException;
};

class EGLErrorException: private EGLException
{
  using EGLException::EGLException;
};

#define checkEglError(message){ \
    EGLint err = eglGetError(); \
    if (err != EGL_SUCCESS) \
    { \
        std::cerr << "EGL Error " << std::hex << err << std::dec << " on line " <<  __LINE__ << std::endl; \
        throw EGLErrorException(message); \
    } \
}

#define checkEglReturn(x, message){ \
    if (x != EGL_TRUE) \
    { \
        std::cerr << "EGL returned not true on line " << __LINE__ << std::endl; \
        throw EGLReturnException(message); \
    } \
}

static inline const char * GetGLErrorString(GLenum error)
{
	const char *str;
	switch( error )
	{
		case GL_NO_ERROR:
			str = "GL_NO_ERROR";
			break;
		case GL_INVALID_ENUM:
			str = "GL_INVALID_ENUM";
			break;
		case GL_INVALID_VALUE:
			str = "GL_INVALID_VALUE";
			break;
		case GL_INVALID_OPERATION:
			str = "GL_INVALID_OPERATION";
			break;
#ifdef __gl_h_
		case GL_STACK_OVERFLOW:
			str = "GL_STACK_OVERFLOW";
			break;
		case GL_STACK_UNDERFLOW:
			str = "GL_STACK_UNDERFLOW";
			break;
		case GL_OUT_OF_MEMORY:
			str = "GL_OUT_OF_MEMORY";
			break;
		case GL_TABLE_TOO_LARGE:
			str = "GL_TABLE_TOO_LARGE";
			break;
#endif
#if GL_EXT_framebuffer_object
		case GL_INVALID_FRAMEBUFFER_OPERATION_EXT:
			str = "GL_INVALID_FRAMEBUFFER_OPERATION_EXT";
			break;
#endif
		default:
			str = "(ERROR: Unknown Error Enum)";
			break;
	}
	return str;
}

#define printGLError(){ \
	GLenum  err = glGetError();; \
    if (err != GL_NO_ERROR) \
    { \
    	const char *str = GetGLErrorString(err); \
		std::cout << "error: " << str << std::endl; \
    } \
}
#endif

// #ifndef NDEBUG
#   define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
// #else
// #   define ASSERT(condition, message) do { } while (false)
// #endif


class cSimAdapter {
public:
	cSimAdapter(std::vector<std::string> args);
	virtual ~cSimAdapter();

	virtual void init();
	virtual void initEpoch();
	virtual void reload();
	virtual double updateAction(std::vector<double> action);
	virtual double updateLLCAction(std::vector<double> action);
	/// Perform on simulation update
	virtual void update();
	virtual void finish();
	/// Returns the reward for executing action act
	virtual void act(std::vector<double> act);

	virtual double calcReward();
	virtual double calcVelocity() const;
	virtual std::vector<double> calcVelocity3D() const;
	virtual bool hasStumbled();
	virtual double jointTorque();

	virtual bool endOfEpoch();
	virtual bool agentHasFallen();
	/// check whether or not the last action has completed and a new action is needed
	virtual bool needUpdatedAction();

	/// Stuff for interacting with simulation
	virtual void setRender(bool shouldRender);
	virtual void display();
	virtual void reshapeScreen(int w, int h);
	/// Interactive functions to doing things in the simulation, like reseting and throwing objects at the character
	virtual void onKeyEvent(int key, int mouseX, int mouseY);
	/// specify the relative path to TerrainRL
	virtual void setRelativeFilePath(std::string relativePath);
	/// specify the relative path to TerrainRL
	virtual void setRandomSeed(int seed);

	virtual std::vector<double> getState() const;
	virtual std::vector<double> getLLCState();
	virtual std::vector<double> getImitationState() const;
	virtual std::vector<double> getImitationStateAtTime(double animTime);
	virtual std::vector<double> getImitationVelocity3D() const;

	/// New stuff for multi agent simulation
	virtual size_t getNumAgents();
	virtual std::shared_ptr<cSimCharacter> getAgent(size_t agent_num);
	virtual double updateActionForAgent(size_t agent_num, std::vector<double> action);
	virtual double updateLLCActionForAgent(size_t agent_num, std::vector<double> action);
	/// Returns the reward for executing action act
	virtual void actForAgent(std::vector<double> act);
	virtual double calcRewardForAgent(size_t agent_num);
	virtual double calcVelocity(size_t agent_num);
	virtual bool hasStumbledForAgent(size_t agent_num);
	virtual double jointTorque(size_t agent_num);
	virtual bool endOfEpochForAgent(size_t agent_num);
	virtual bool agentHasFallenForAgent(size_t agent_num);
	virtual bool needUpdatedActionForAgent(size_t agent_num);
	virtual std::vector<double> getStateForAgent(size_t agent_num);
	virtual std::vector<double> getLLCStateForAgent(size_t agent_num);

	virtual size_t getActionSpaceSize() const;
	virtual std::vector<std::vector<double> > getActionSpaceBounds() const;
	virtual size_t getObservationSpaceSize() const;

	virtual void changeAnimTimestep(double timestep);
	virtual double getAnimTimestep() const;
	virtual double getAnimationTime() const;

	virtual void setSimState( std::vector<double> state_);
	virtual std::vector<double> getSimState() const;
	std::vector<double> getJointWeights() const;

	virtual void setDesiredVel(double vel);
	virtual size_t getTaskID() const;
	virtual void setTaskID(size_t task);
	virtual size_t GetNumTasks() const;

	void handleUpdatedAction();

	/// New rendering stuff
	virtual void setHeadlessRender(bool hr);
	std::vector<unsigned char> getPixels(size_t x_start, size_t y_start, size_t width, size_t height);
	virtual void setRenderingGPUDevicveIndex(int desiredGPUDeviceIndex_);

#ifdef USE_OpenGLES
	/*
		 * Create an RGB, double-buffered Headless window.
		 * context handle.
		 */
		virtual void make_headless_window(EGLDisplay egl_dpy,
					  const char *name,
					  int x, int y, int width, int height,
					  EGLContext *ctxRet,
					  EGLSurface *surfRet);

		virtual EGLDisplay eglGetDisplay_(NativeDisplayType nativeDisplay=EGL_DEFAULT_DISPLAY);

		int m_frameCount;
		EGLSurface egl_surf;
		EGLContext egl_ctx;
		EGLDisplay egl_dpy;
		char *dpyName = NULL;
		GLboolean printInfo = GL_FALSE;
		EGLint egl_major, egl_minor;
		int i;
		const char *s;

		void create_shaders(void);
		bool drawAgent = true, drawObject = true;
		GLfloat view_rotx = 0.0, view_roty = 0.0;
		GLfloat view_transx = 0.0, view_transy = 0.0, view_transz = 0.0;
		GLfloat view_transx2 = 0.0, view_transy2 = 0.0, view_transz2 = 0.0;
		GLfloat camPos[3] = {0.0, 0.0, 0.0};

		GLint u_matrix = -1;
		GLint attr_pos = 0, attr_color = 1;

		virtual void setPosition(float xs, float ys, float zs);
		virtual void setPosition2(float xs, float ys, float zs);
		virtual void setCameraPosition(float xs, float ys, float zs);

		virtual void setDrawAgent(bool draw_);
		virtual void setDrawObject(bool draw_);
#endif


protected:
	size_t _lastControllerState;
	bool _render;
	std::shared_ptr<cScenario> _gScenario;
	std::shared_ptr<cScenarioSimChar> _scene;
	std::vector<std::string> _args;
	std::string _relativePath;

	/// New rendering stuff
	/*
   EGLSurface _egl_surf;
   EGLContext _egl_ctx;
   EGLDisplay _egl_dpy;
   Display* _dpy;
   GLXPbuffer _drawable;
	*/
   bool _headless_render;
   int desiredGPUDeviceIndex;
};

#endif /* SIMADAPTER_H_ */
