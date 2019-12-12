/*
 * SimAdapter.cpp
 *
 *  Created on: 2017-01-20
 *      Author: gberseth
 */

#include "SimAdapter.h"
#include "Main.h"
#include "sim/TerrainRLCharController.h"
#include "sim/WaypointController.h"
#include "anim/KinSimCharacter.h"
#include "scenarios/ScenarioExp.h"
#include "scenarios/DrawScenarioImitateEval.h"
#include "scenarios/DrawScenarioImitateVizEval.h"
#include "scenarios/ScenarioImitate.h"
#include "scenarios/ScenarioImitateEval.h"
#include "scenarios/ScenarioImitateVizEval.h"
#include "scenarios/ScenarioImitateStepEval.h"
#include "scenarios/ScenarioExp.h"
#include "scenarios/ScenarioHikeEval.h"
#include "scenarios/ScenarioSpaceMultChar.h"
#include "scenarios/ScenarioTestCaseMultChar.h"
#include "scenarios/DrawScenarioTestCaseMultChar.h"
#include "scenarios/ScenarioMultChar.h"
#include "scenarios/ScenarioMultCharConcentricCircle.h"
#include "scenarios/DrawScenarioMultChar.h"
#include "scenarios/ScenarioSoccerEval.h"
#include "scenarios/ScenarioImitateVizEval.h"
#include "scenarios/DrawScenarioMultCharConcentricCircle.h"
#include "scenarios/DrawScenarioMultiTaskImitateVizEval.h"
#include "scenarios/ScenarioMultiTaskImitateVizEval.h"
#include "scenarios/DrawScenarioMultCharRugby.h"
#include "scenarios/ScenarioMultCharRugby.h"
#include "scenarios/DrawScenarioImitateEvalMultiTask.h"
#include "scenarios/ScenarioImitateEvalMultiTask.h"

cSimAdapter::cSimAdapter(std::vector<std::string> args) {
	// TODO Auto-generated constructor stub
	_lastControllerState=0;
	_render=false;
	_gScenario = nullptr;
	_args = args;
	this->_headless_render=false;
	this->_relativePath = "";
#ifdef USE_OpenGLES
	m_frameCount = 0;
	dpyName = NULL;
	printInfo = GL_FALSE;
	desiredGPUDeviceIndex = 0;

	drawAgent = true, drawObject = false;

	view_rotx = 0.0, view_roty = 0.0;
	view_transx = 0.0, view_transy = 0.0, view_transz = 0.0;
	view_transx2 = 0.0, view_transy2 = 0.0, view_transz2 = 0.0;
	setCameraPosition(0.0, 0.0, 0.0);

	u_matrix = -1;
	attr_pos = 0, attr_color = 1;
#endif
}

cSimAdapter::~cSimAdapter() {
	// TODO Auto-generated destructor stub
}

#ifdef USE_OpenGLES
void cSimAdapter::setPosition(float xs, float ys, float zs)
{
	view_transx = xs;
	view_transy = ys;
	view_transz = zs;
}

void cSimAdapter::setPosition2(float xs, float ys, float zs)
{
	view_transx2 = xs;
	view_transy2 = ys;
	view_transz2 = zs;
}

void cSimAdapter::setCameraPosition(float xs, float ys, float zs)
{
	camPos[0] = xs;
	camPos[1] = ys;
	camPos[2] = zs;
}

void cSimAdapter::setDrawAgent(bool draw_)
{
	drawAgent = draw_;
}
void cSimAdapter::setDrawObject(bool draw_)
{
	drawObject = draw_;
}
#endif

void cSimAdapter::setRenderingGPUDevicveIndex(int desiredGPUDeviceIndex_)
{
	desiredGPUDeviceIndex = desiredGPUDeviceIndex_;
}

void cSimAdapter::setRender(bool shouldRender)
{
	this->_render=shouldRender;
}
void cSimAdapter::setHeadlessRender(bool hr)
{
	this->_headless_render = hr;
}

#ifdef USE_OpenGLES
/*
 * Create an RGB, double-buffered Headless window.
 * context handle.
 */
void cSimAdapter::make_headless_window(EGLDisplay egl_dpy,
              const char *name,
              int x, int y, int width, int height,
              EGLContext *ctxRet,
              EGLSurface *surfRet)
{
	/*
    const EGLint attribs[] ={
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         24,
        EGL_STENCIL_SIZE,       8,
        EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_NONE
    };
    */

   static const EGLint attribs[] = {
      EGL_RED_SIZE, 1,
      EGL_GREEN_SIZE, 1,
      EGL_BLUE_SIZE, 1,
      EGL_DEPTH_SIZE, 1,
      EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
      EGL_NONE
   };

#if USE_FULL_GL
   static const EGLint ctx_attribs[] = {
       EGL_NONE
   };
#else
   static const EGLint ctx_attribs[] = {
      EGL_CONTEXT_CLIENT_VERSION, 2,
      EGL_NONE
   };
#endif

   unsigned long mask;
   int num_visuals;
   EGLContext ctx;
   EGLConfig config;
   EGLint num_configs;
   EGLint vid;


   if (!eglChooseConfig( egl_dpy, attribs, &config, 1, &num_configs)) {
      printf("Error: couldn't get an EGL visual config\n");
      exit(1);
   }

   assert(num_configs > 0);
   assert(config);

   if (!eglGetConfigAttrib(egl_dpy, config, EGL_NATIVE_VISUAL_ID, &vid)) {
      printf("Error: eglGetConfigAttrib() failed\n");
      exit(1);
   }

#if USE_FULL_GL /* XXX fix this when eglBindAPI() works */
   eglBindAPI(EGL_OPENGL_API);
#else
   eglBindAPI(EGL_OPENGL_ES_API);
#endif

   ctx = eglCreateContext(egl_dpy, config, EGL_NO_CONTEXT, ctx_attribs );
   if (!ctx) {
      printf("Error: eglCreateContext failed\n");
      exit(1);
   }

#if !USE_FULL_GL
   /* test eglQueryContext() */
   {
      EGLint val;
      eglQueryContext(egl_dpy, ctx, EGL_CONTEXT_CLIENT_VERSION, &val);
      printf("qeurry EGL_CONTEXT_CLIENT_VERSION %d\n", val);
      assert(val != 0);
   }
#endif
   /// Could not get A WindowSurface to work
   // *surfRet = eglCreateWindowSurface(egl_dpy, config, (EGLNativeWindowType)NULL, NULL);
   static const EGLint pbufferAttribs[] = {
         EGL_WIDTH, width,
         EGL_HEIGHT, height,
         EGL_NONE,
   };
   *surfRet = eglCreatePbufferSurface(egl_dpy, config,
                                                      pbufferAttribs);
   if (!*surfRet) {
	  std::cout << glGetError() << std::endl;
      printf("Error: eglCreateWindowSurface failed\n");
      exit(1);
   }

   /* sanity checks */
   {
      EGLint val;
      eglQuerySurface(egl_dpy, *surfRet, EGL_WIDTH, &val);
      assert(val == width);
      eglQuerySurface(egl_dpy, *surfRet, EGL_HEIGHT, &val);
      assert(val == height);
      assert(eglGetConfigAttrib(egl_dpy, config, EGL_SURFACE_TYPE, &val));
      // assert(val & EGL_WINDOW_BIT);
      assert(val);
   }

   *ctxRet = ctx;
}

EGLDisplay cSimAdapter::eglGetDisplay_(NativeDisplayType nativeDisplay)
{
  EGLDisplay eglDisplay = eglGetDisplay(nativeDisplay);
  checkEglError("Failed to Get Display: eglGetDisplay");
  std::cerr << "Failback to eglGetDisplay" << std::endl;
  return eglDisplay;
}

void cSimAdapter::create_shaders(void)
{
   const char *fragShaderText =
      "precision mediump float;\n"
      "varying vec4 v_color;\n"
      "void main() {\n"
      "   gl_FragColor = v_color;\n"
      "}\n";
   const char *vertShaderText =
      "uniform mat4 modelviewProjection;\n"
      "attribute vec4 pos;\n"
      "attribute vec4 color;\n"
      "varying vec4 v_color;\n"
      "void main() {\n"
      "   gl_Position = modelviewProjection * pos;\n"
      "   v_color = color;\n"
      "}\n";

   GLuint fragShader, vertShader, program;
   GLint stat;

   fragShader = glCreateShader(GL_FRAGMENT_SHADER);
   std::cout << "Created frag shader: " << fragShader << std::endl;
   checkEglError("Create frag shader");
   printGLError();
   glShaderSource(fragShader, 1, (const char **) &fragShaderText, NULL);
   checkEglError("Get Shader source");
   printGLError();
   glCompileShader(fragShader);
   checkEglError("Compile Shader source");
   printGLError();
   glGetShaderiv(fragShader, GL_COMPILE_STATUS, &stat);
   if (!stat) {
      printf("Error: fragment shader did not compile!\n");
      checkEglError("Get Shader compile status");
      printGLError();
      exit(1);
   }

   vertShader = glCreateShader(GL_VERTEX_SHADER);
   std::cout << "Created vert shader: " << vertShader << std::endl;
   glShaderSource(vertShader, 1, (const char **) &vertShaderText, NULL);
   glCompileShader(vertShader);
   glGetShaderiv(vertShader, GL_COMPILE_STATUS, &stat);
   if (!stat) {
      printf("Error: vertex shader did not compile!\n");
      exit(1);
   }

   program = glCreateProgram();
   glAttachShader(program, fragShader);
   glAttachShader(program, vertShader);
   glLinkProgram(program);
   std::cout << "Linked shader program : " << program << std::endl;

   glGetProgramiv(program, GL_LINK_STATUS, &stat);
   if (!stat) {
      char log[1000];
      GLsizei len;
      glGetProgramInfoLog(program, 1000, &len, log);
      printf("Error: linking:\n%s\n", log);
      exit(1);
   }

   glUseProgram(program);

   if (1) {
      /* test setting attrib locations */
      glBindAttribLocation(program, attr_pos, "pos");
      glBindAttribLocation(program, attr_color, "color");
      glLinkProgram(program);  /* needed to put attribs into effect */
   }
   else {
      /* test automatic attrib locations */
      attr_pos = glGetAttribLocation(program, "pos");
      attr_color = glGetAttribLocation(program, "color");
   }

   u_matrix = glGetUniformLocation(program, "modelviewProjection");
   printf("Uniform modelviewProjection at %d\n", u_matrix);
   printf("Attrib pos at %d\n", attr_pos);
   printf("Attrib color at %d\n", attr_color);
}

#endif

void cSimAdapter::init()
{
	// testBVHReader();
    char** cstrings = new char*[_args.size()];
    for(size_t i = 0; i < _args.size(); ++i)
    {
        cstrings[i] = new char[_args[i].size() + 1];
        std::strcpy(cstrings[i], _args[i].c_str());
    }

	int argc = _args.size();
	// char* argv[] = cstrings;

 	gArgc = argc;
	gArgv = cstrings;
	ParseArgs(gArgc, gArgv);

	std::string scenario_name = "";
	gArgParser->ParseString("scenario", scenario_name);

	if (scenario_name == "hike_eval"
			|| scenario_name == "space_mult_char"
			|| scenario_name == "multi_char"
			|| scenario_name == "multi_char_circle"
					)
	{
		gCameraPosition = tVector(0, 50, 100, 0);
	}

	// InitCaffe();

	if (_render)
	{
#ifdef USE_OpenGLES
		if (this->_headless_render)
		{

		   std::cout << "Desired device: " << desiredGPUDeviceIndex << std::endl;

		   PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)eglGetProcAddress("eglQueryDevicesEXT");
		   checkEglError("Failed to get EGLEXT: eglQueryDevicesEXT");
		   PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
		   checkEglError("Failed to get EGLEXT: eglGetPlatformDisplayEXT");
		   PFNEGLQUERYDEVICEATTRIBEXTPROC eglQueryDeviceAttribEXT = (PFNEGLQUERYDEVICEATTRIBEXTPROC)eglGetProcAddress("eglQueryDeviceAttribEXT");
		   checkEglError("Failed to get EGLEXT: eglQueryDeviceAttribEXT");

		   if (desiredGPUDeviceIndex >= 0)
			 {
			   EGLDeviceEXT *eglDevs;
			   EGLint numberDevices;

			   //Get number of devices
			   checkEglReturn(
				 eglQueryDevicesEXT(0, NULL, &numberDevices),
				 "Failed to get number of devices. Bad parameter suspected"
			   );
			   checkEglError("Error getting number of devices: eglQueryDevicesEXT");

			   std::cerr << numberDevices << " devices found" << std::endl;

			   if (numberDevices)
			   {
				 EGLAttrib cudaIndex;

				 //Get devices
				 eglDevs = new EGLDeviceEXT[numberDevices];
				 checkEglReturn(
				   eglQueryDevicesEXT(numberDevices, eglDevs, &numberDevices),
				   "Failed to get devices. Bad parameter suspected"
				 );
				 checkEglError("Error getting number of devices: eglQueryDevicesEXT");

				 /// Need to perform this search because on the compute server all devices appear available
				 /// Need to find the true one that was allocated, using /dev/nvidia* is the only way.
				 size_t avialalbe_device_index = 0;
				 for(i=0; i<numberDevices; i++)
				 {
					 egl_dpy = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT, eglDevs[i], 0);
							   checkEglError("Error getting Platform Display: eglGetPlatformDisplayEXT");
							   std::cerr << "Got GPU device " << i << std::endl;

					  if (!egl_dpy) {
						  std::cerr <<  "Error: eglGetDisplay() failed" << std::endl;
					  return;
					 }
					  std::string fileName = "/dev/nvidia";
					  if (file_exists_test(fileName + std::to_string(i)))
					  {
						  if (avialalbe_device_index == desiredGPUDeviceIndex)
						  { // Use this device
							  std::cout << "Found GPU device " << "/dev/nvidia" << i << " for rendering" << std::endl;
							  break;
						  }
						  avialalbe_device_index++;
					  }
					  /*
					 if (!eglInitialize(egl_dpy, &egl_major, &egl_minor)) {
						 std::cerr << "Error: eglInitialize() failed on device: " << i << std::endl;
					  continue;
					 }
					 else
					 { /// Found a device that works.
						 break;
					 }
					   */
				 }
				 if (!egl_dpy)
				 {
				   egl_dpy = eglGetDisplay_();
				 }
			   }
			   else
			   {//If no devices were found, or a matching cuda not found, get a Display the normal way
				 egl_dpy = eglGetDisplay_();
			   }
			 }
			 else
			 {
			   egl_dpy = eglGetDisplay_();
			 }

			 if (egl_dpy == EGL_NO_DISPLAY)
			   throw EGLException("No Disply Found");
		   // unsetenv("DISPLAY"); //Force Headless
			// egl_dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
		   if (!egl_dpy) {
			  printf("Error: eglGetDisplay() failed\n");
			  return;
		   }

		   if (!eglInitialize(egl_dpy, &egl_major, &egl_minor)) {
			  printf("Error: eglInitialize() failed\n");
			  return;
		   }

		   s = eglQueryString(egl_dpy, EGL_VERSION);
		   printf("EGL_VERSION = %s\n", s);

		   s = eglQueryString(egl_dpy, EGL_VENDOR);
		   printf("EGL_VENDOR = %s\n", s);

		   s = eglQueryString(egl_dpy, EGL_EXTENSIONS);
		   printf("EGL_EXTENSIONS = %s\n", s);

		   s = eglQueryString(egl_dpy, EGL_CLIENT_APIS);
		   printf("EGL_CLIENT_APIS = %s\n", s);

		   make_headless_window(egl_dpy,
						 "OpenGL ES 2.x tri", 0, 0, gWinWidth, gWinHeight,
						&egl_ctx, &egl_surf);

		   if (!eglMakeCurrent(egl_dpy, egl_surf, egl_surf, egl_ctx)) {
			  printf("Error: eglMakeCurrent() failed\n");
			  return;
		   }

		   if (printInfo) {
			  printf("GL_RENDERER   = %s\n", (char *) glGetString(GL_RENDERER));
			  printf("GL_VERSION    = %s\n", (char *) glGetString(GL_VERSION));
			  printf("GL_VENDOR     = %s\n", (char *) glGetString(GL_VENDOR));
			  printf("GL_EXTENSIONS = %s\n", (char *) glGetString(GL_EXTENSIONS));
		   }

			typedef void (*proc)();

			#if 1 /* test code */
			proc p = eglGetProcAddress("glMapBufferOES");
			assert(p);
			#endif
/*
			GLuint fragShader, vertShader, program;
			fragShader = glCreateShader(GL_FRAGMENT_SHADER);
			checkEglError("Create frag shader");
			checkEglError("Create frag shader");
			std::cout << "Created fragment shader: " << fragShader << std::endl;
			vertShader = glCreateShader(GL_FRAGMENT_SHADER);
			checkEglError("Create vert shader");
			checkEglError("Create vert shader");
			std::cout << "Created vertex shader: " << vertShader << std::endl;
			*/
			glClearColor(0.9, 0.9, 0.9, 0.0);
			create_shaders();

			glViewport(0, 0, (GLint) gWinWidth, (GLint) gWinHeight);
			InitOpenGl();
		}
		// else
#else
		{

			glutInit(&gArgc, gArgv);
			glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
			glutInitWindowSize(gWinWidth, gWinHeight);
			glutCreateWindow("Terrain RL");
			InitOpenGl();
		}
#endif
	}
	// SetupScenario();
	// ClearScenario();

	// if (scenario_name == "sim_char")
	if ( _render )
	{
		InitCamera();
		ClearScenario();

		if (scenario_name == "imitate_eval")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateEval>(new cDrawScenarioImitateEval(gCamera));
			// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}if (scenario_name == "imitate_eval_multitask")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateEvalMultiTask>(new cDrawScenarioImitateEvalMultiTask(gCamera));
			// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "sim_char")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "imitate_step_eval")
		{
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateStepEval>(new cDrawScenarioImitateStepEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "hike_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);

			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioHikeEval>(new cDrawScenarioHikeEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "space_mult_char")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSpaceMultChar>(new cDrawScenarioSpaceMultChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "multi_char_rugby")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioMultCharRugby>(new cDrawScenarioMultCharRugby(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "testcase_mult_char")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioTestCaseMultChar>(new cDrawScenarioTestCaseMultChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "multi_char")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioMultChar>(new cDrawScenarioMultChar(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "multi_char_circle")
				{
					gCameraPosition = tVector(0, 100, 100, 0);
					std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioMultCharConcentricCircle>(new cDrawScenarioMultCharConcentricCircle(gCamera));
					this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
					this->_gScenario = scenario__;
					if (this->_gScenario != NULL)
					{
						auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
						if (sim_char_scene != nullptr)
						{
							sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
						}

					}
					gScenario = std::shared_ptr<cDrawScenario>(scenario__);
					this->_gScenario = gScenario;
				}
		else if (scenario_name == "soccer_eval")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSoccerEval>(new cDrawScenarioSoccerEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "imitate_viz_eval")
		{
			gCameraPosition = tVector(0, 40, 40, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioImitateVizEval>(new cDrawScenarioImitateVizEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "multitask_imitate_viz")
		{
			gCameraPosition = tVector(0, 40, 40, 0);
			std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioMultiTaskImitateVizEval>(new cDrawScenarioMultiTaskImitateVizEval(gCamera));
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
			this->_gScenario = scenario__;
			if (this->_gScenario != NULL)
			{
				auto sim_char_scene = std::dynamic_pointer_cast<cDrawScenarioTerrainRL>(scenario__);
				if (sim_char_scene != nullptr)
				{
					sim_char_scene->SetOutputTex(gIntermediateFrameBuffer);
				}

			}
			gScenario = std::shared_ptr<cDrawScenario>(scenario__);
			this->_gScenario = gScenario;
		}
		else if (scenario_name == "headless_test")
		{

		}
		else
		{
			std::cerr << "scenario not recognized: " << scenario_name << std::endl;
			exit(-1);
		}
	}
	// else if (scenario_name == "sim_char_render")
	else
	{
		ClearScenario();
		if (scenario_name == "imitate_eval")
		{
			// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSimChar>(new cScenarioImitateEval() );
			this->_scene = std::dynamic_pointer_cast<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "imitate_eval")
		{
			// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSimChar>(new cScenarioImitateEvalMultiTask() );
			this->_scene = std::dynamic_pointer_cast<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "sim_char")
		{
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSimChar>(new cScenarioSimChar());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "imitate_step_eval")
		{
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioImitateStepEval>(new cScenarioImitateStepEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "hike_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioHikeEval>(new cScenarioHikeEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			// gScenario = std::shared_ptr<cScenario>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "space_mult_char")
		{
			gCameraPosition = tVector(0, 100, 100, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSpaceMultChar>(new cScenarioSpaceMultChar());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "multi_char_rugby")
		{
			gCameraPosition = tVector(0, 200, 200, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioMultCharRugby>(new cScenarioMultCharRugby());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "testcase_mult_char")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioTestCaseMultChar>(new cScenarioTestCaseMultChar());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "multi_char")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioMultChar>(new cScenarioMultChar());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "multi_char_circle")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioMultCharConcentricCircle>(new cScenarioMultCharConcentricCircle());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "soccer_eval")
		{
			gCameraPosition = tVector(0, 30, 30, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioSoccerEval>(new cScenarioSoccerEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "imitate_viz_eval")
		{
			gCameraPosition = tVector(0, 0, 40, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioImitateVizEval>(new cScenarioImitateVizEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "multitask_imitate_viz")
		{
			gCameraPosition = tVector(0, 0, 40, 0);
			std::shared_ptr<cScenarioSimChar> scenario__ = std::shared_ptr<cScenarioMultiTaskImitateVizEval>(new cScenarioMultiTaskImitateVizEval());
			this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__);
			this->_gScenario = scenario__ ;
		}
		else if (scenario_name == "headless_test")
		{

		}
		else
		{
			std::cout << "Scenario type not yet supported by adapter: " << scenario_name << std::endl;

		}
		// gScenario = this->_gScenario;
	}

	this->_gScenario->ParseArgs(gArgParser);
	this->_gScenario->Init();
	printf("Loaded scenario: %s\n", this->_gScenario->GetName().c_str());
	// if ( _render && (!this->_headless_render) )
	if ( _render )
	{
		this->_scene = std::shared_ptr<cScenarioSimChar>(std::dynamic_pointer_cast<cDrawScenarioSimChar>(this->_gScenario)->GetScene());
		if ( !this->_headless_render )
		{
#ifndef USE_OpenGLES
			Reshape(gWinWidth, gWinHeight);
			glutDisplayFunc(Display_);
			glutReshapeFunc(Reshape);
			glutKeyboardFunc(Keyboard);
			glutMouseFunc(MouseClick);
			glutMotionFunc(MouseMove);
			// glutTimerFunc(gDisplayAnimTime, Animate, 0);
#endif
		}
	}

	InitTime();
	// glutMainLoop();

	for(size_t i = 0; i < _args.size(); ++i)
	{
		// delete[] cstrings[i];
	}
	// return EXIT_SUCCESS;
	// Zoom out a little for viz environments
	double zoom = -1.2;
	//gCamera.SetPosition(tVector(0, 75, -150, 0));
	tVector focus = gCamera.GetFocus();
	tVector cam_offset = -(gCamera.GetFocus() - gCamera.GetPosition());
	double w = gCamera.GetWidth();
	double h = gCamera.GetHeight();

	double delta_scale = 1 - zoom;
	tVector delta = cam_offset * delta_scale;
	gCamera.SetPosition(focus + delta);
	w *= delta_scale;
	h *= delta_scale;
	gCamera.Resize(w, h);
	gCamera.SetFocus(focus);
}

void cSimAdapter::initEpoch()
{
	if ( this->_gScenario != nullptr)
	{
		this->_gScenario->Reset();
	}
	gForceClear = true;
}

void cSimAdapter::reload()
{
	if ( this->_gScenario != nullptr)
	{
		this->init();
	}
	gForceClear = true;
}

double cSimAdapter::getAnimTimestep() const
{
	return gAnimStep;
}
void cSimAdapter::changeAnimTimestep(double timestep)
{
	gAnimStep = timestep;
	std::cout << "timestep changed to " << gAnimStep << std::endl;
}

void cSimAdapter::update()
{

	if (gAnimate)
	{
		if ( _render && !this->_headless_render )
		{
#ifndef USE_OpenGLES
			int num_steps = GetNumTimeSteps();
			int current_time = glutGet(GLUT_ELAPSED_TIME);
			int elapsedTime = current_time - gPrevTime;

			gPrevTime = current_time;

			double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
			// elapsedTime = int( 1000 * timestep );

			/*
			if (this->_gScenario != NULL)
			{

				for (size_t i = 0; i < num_steps; ++i)
				{
					this->_gScenario->Update(timestep);
				}
			}
			*/

			for (int i = 0; i < num_steps; ++i)
			{
				Update(timestep);
			}

			int update_dur = glutGet(GLUT_ELAPSED_TIME) - current_time;

			// glutPostRedisplay();
			gUpdatesPerSec = (num_steps / (elapsedTime * 0.001));
			int timer_step = CalcDisplayAnimTime();
			timer_step -= update_dur;
			timer_step = std::max(timer_step, 0);

			// glutTimerFunc(timer_step, Animate, 0);
			// Animate(100);
#endif
		}
		else
		{
			int num_steps = GetNumTimeSteps();

			double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
			timestep = timestep;

			if (this->_gScenario != NULL)
			{

				for (size_t i = 0; i < num_steps; ++i)
				{
					this->_gScenario->Update(timestep);
				}
			}
		}
	}

	if (this->_gScenario != nullptr)
	{
		if (this->_gScenario->IsDone())
		{
			Shutdown();
		}
	}
	/*
	if ( _render )
	{
		Display();
	}
	*/
}


void cSimAdapter::display()
{
	if ( _render )
	{
		if (this->_headless_render)
		{
			// glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			// eglSwapBuffers(_egl_dpy, _egl_surf);
			/*
			glUseProgram(3);
			float tri_scale = 1.0;
			   const GLfloat verts[3][2] = {
				  { -1, -1 },
				  {  1, -1 },
				  {  0,  1 }
			   };
			   GLfloat colors[3][3] = {
				  { 0, 0, 1 },
				  { 0, 0, 1 },
				  { 0, 0, 1 }
			   };
			   GLfloat mat[16], rot[16], scale[16], trans[16], camMat[16];
			   GLfloat mat2[16], rot2[16], scale2[16], trans2[16];
			   make_identity_matrix(mat);
			   make_identity_matrix(mat2);
			   /// Set modelview/projection matrix
			   make_z_rot_matrix(view_rotx, rot);
			   make_scale_matrix(tri_scale, tri_scale, tri_scale, scale);
			   make_translation_matrix(view_transx, view_transy, view_transz, trans);
			   make_translation_matrix(-camPos[0],-camPos[1], -camPos[2], camMat);
			   // mul_matrix(mat, trans, rot);
			   mul_matrix(mat, mat, scale);
			   mul_matrix(mat, mat, trans);
			   mul_matrix(mat, mat, camMat);
			   mul_matrix(mat, mat, scale);


				make_z_rot_matrix(view_rotx, rot2);
				make_scale_matrix(tri_scale, tri_scale, tri_scale, scale2);
				make_translation_matrix(view_transx2, view_transy2, view_transz2, trans2);
				// mul_matrix(mat2, trans2, rot2);
				mul_matrix(mat2, mat2, scale2);
				mul_matrix(mat2, mat2, trans2);
				mul_matrix(mat2, mat2, camMat);
				mul_matrix(mat2, mat2, scale2);

			   glUniformMatrix4fv(u_matrix, 1, GL_FALSE, mat);

			   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			   if (drawAgent)
			   {
				  glVertexAttribPointer(attr_pos, 2, GL_FLOAT, GL_FALSE, 0, verts);
				  glVertexAttribPointer(attr_color, 3, GL_FLOAT, GL_FALSE, 0, colors);
				  glEnableVertexAttribArray(attr_pos);
				  glEnableVertexAttribArray(attr_color);

				  glDrawArrays(GL_TRIANGLES, 0, 3);

				  glDisableVertexAttribArray(attr_pos);
				  glDisableVertexAttribArray(attr_color);
			   }

			   glUniformMatrix4fv(u_matrix, 1, GL_FALSE, mat2);
			   GLfloat colors2[3][3] = {
					 { 1, 0, 0 },
					 { 1, 0, 0 },
					 { 1, 0, 0 }
				  };
			   if (drawObject)
			   {
				  glVertexAttribPointer(attr_pos, 2, GL_FLOAT, GL_FALSE, 0, verts);
				  glVertexAttribPointer(attr_color, 3, GL_FLOAT, GL_FALSE, 0, colors2);
				  glEnableVertexAttribArray(attr_pos);
				  glEnableVertexAttribArray(attr_color);

				  glDrawArrays(GL_TRIANGLES, 0, 3);

				  glDisableVertexAttribArray(attr_pos);
				  glDisableVertexAttribArray(attr_color);
			   }
				eglSwapBuffers(egl_dpy, egl_surf);
				*/
#ifdef USE_OpenGLES
			// UpdateIntermediateBuffer();

			cDrawUtil::MatrixMode(Util::PROJECTION_MAT);
			SetupCamProjection();

			cDrawUtil::MatrixMode(Util::MODELVIEW_MAT);
			cDrawUtil::LoadIdentity();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			// gIntermediateFrameBuffer->BindBuffer();
			// ClearFrame();
			DrawScene();
			// DrawInfo();
			// gIntermediateFrameBuffer->UnbindBuffer();

			// CopyFrame();

			// glutSwapBuffers();
			// glXSwapBuffers(_dpy, _drawable);
			// gDispalyPrevTime = glutGet(GLUT_ELAPSED_TIME);
			eglSwapBuffers(egl_dpy, egl_surf);
			gReshaping = false;
#else
			Display_();
#endif
		}
		else
		{
#ifndef USE_OpenGLES
			Display_();
#endif
		}
	}
}

// std::vector<std::vector<std::vector<unsigned char> > > EGLRender::getPixels(size_t x_start, size_t y_start, size_t width, size_t height)
std::vector<unsigned char> cSimAdapter::getPixels(size_t x_start, size_t y_start, size_t width, size_t height)
{
	// drawAgent = true;
	// drawObject = false;
	// draw();
	// std::vector<std::vector<std::vector<unsigned char> > > out(height,
	// 		std::vector<std::vector<unsigned char> >(width,
	// 				std::vector<unsigned char>(3, 0)));
	// std::vector<unsigned char> out;
	size_t num_pixels = 3*width*height;
	std::vector<unsigned char> out(num_pixels, 0);
	unsigned char m_pixels[num_pixels];
	glReadPixels(x_start, y_start, width, height,
			GL_RGB,GL_UNSIGNED_BYTE, (GLvoid *) out.data());
			// GL_RGB,GL_UNSIGNED_BYTE, (GLvoid *) m_pixels);
/*
	for (size_t h = 0; h < height; h ++)
	{
		for (size_t w = 0; w < width; w ++)
		{
			for (size_t c = 0; c < 3; c ++)
			{
				// col.push_back(m_pixels[(c * width * height) + (h*height) + w]);
				// 						gets row  gets column   get colour

				out[h][w][c] = m_pixels[(h*(width*3)) + (w*3) + c];
			}
		}
	}
*/
	return out;
}


void cSimAdapter::finish()
{
	if (this->_gScenario != nullptr)
	{
		if (this->_headless_render)
		{
		  //  eglDestroyContext(_egl_dpy, _egl_ctx);
		  //  eglDestroySurface(_egl_dpy, _egl_surf);
		  //  eglTerminate(_egl_dpy);
			Shutdown();
		}
		else
		{
			Shutdown();

		}
	}
}

double cSimAdapter::calcReward()
{

	// std::shared_ptr<cScenarioExp> tmp_scenario = std::static_pointer_cast< cScenarioExp>(this->_gScenario);
	double reward_ = this->_scene->CalcReward();
	/*
	std::cout << "Reward: " << r << std::endl;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	double reward_ = controller->CalcReward();
	*/
	return reward_;
}

double cSimAdapter::calcVelocity() const
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	return char_->GetRootVel()(0);
}

std::vector<double> cSimAdapter::calcVelocity3D() const
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	std::vector<double> out;
	out.push_back(char_->GetRootVel()(0));
	out.push_back(char_->GetRootVel()(1));
	// if ( controller->Is3D() )
	//{
	out.push_back(char_->GetRootVel()(2));
	//}
	return out;
}

std::vector<double> cSimAdapter::getImitationVelocity3D() const
{
	// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
	std::shared_ptr<cScenarioImitateVizEval> scenario__tmp = std::dynamic_pointer_cast<cScenarioImitateVizEval>(this->_scene);
	if ( scenario__tmp != nullptr)
	{
		Eigen::VectorXd state;
		const std::shared_ptr<cKinSimCharacter> char_ = std::dynamic_pointer_cast<cKinSimCharacter>(scenario__tmp->GetKinChar());
		const std::shared_ptr<cSimCharacter> char__ = this->_scene->GetCharacter();
		std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char__->GetController());
		std::vector<double> out;
		out.push_back(char_->GetRootVel()(0));
		out.push_back(char_->GetRootVel()(1));
		// if ( controller->Is3D() )
		// {
		out.push_back(char_->GetRootVel()(2));
		//}
		return out;
	}
	else
	{
		throw std::runtime_error("This scenario does not support imitation information. " + std::string(__FILE__)+ ":"+ std::to_string(__LINE__));
	}

}

bool cSimAdapter::hasStumbled()
{
	// const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	return this->_scene->HasStumbled();
}

double cSimAdapter::jointTorque()
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	size_t num_joints = char_->GetNumJoints();
	double torque_sum = 0;
	for (size_t i=0; i < num_joints; ++i)
	{

		cJoint& joint = char_->GetJoint(i);
		tVector torque(0, 0, 0, 0);
		if (joint.IsValid())
		{
			torque = joint.GetTotalTorque();
		}
		// std::cout << "Torque: " << torque[2] << std::endl;
		/// Only for 2D for now...
		torque_sum += fabs(torque[2]);

	}
	/// divided by max torque (300)
	torque_sum = (torque_sum / (double)num_joints) / 300.0;
	return torque_sum;
}

double cSimAdapter::updateAction(std::vector<double> act)
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());

	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());
	cTerrainRLCharController::tAction action;
	action.mID;
	action.mParams = Eigen::VectorXd::Zero(act.size());
	for (size_t i=0; i < act.size(); i++)
	{
		action.mParams(i) = act[i];
	}

	controller->ApplyAction(action);
	return 0.0;
}

double cSimAdapter::updateLLCAction(std::vector<double> act)
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	auto controller = std::dynamic_pointer_cast<cWaypointController>(char_->GetController());
		// std::shared_ptr<cWaypointController> controller =  std::static_pointer_cast< cWaypointController >(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();

		cTerrainRLCharController::tAction action;
		action.mID;
		action.mParams = Eigen::VectorXd::Zero(act.size());
		for (size_t i=0; i < act.size(); i++)
		{
			action.mParams(i) = act[i];
		}

		llc->ApplyAction(action);
	}
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	return 0.0;
}

void cSimAdapter::act(std::vector<double> act)
{

	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());
	cTerrainRLCharController::tAction action;
	action.mID;
	action.mParams = Eigen::VectorXd::Zero(act.size());
	for (size_t i=0; i < act.size(); i++)
	{
		action.mParams(i) = act[i];
	}

	controller->ApplyAction(action);

}

std::vector<double> cSimAdapter::getState() const
{
	Eigen::VectorXd state;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	// const std::shared_ptr<cTerrainRLCharController>& controller =  static_cast< const std::shared_ptr<cTerrainRLCharController>& >(char_->GetController());
	// controller = char->getCOntroller()
	controller->ParseGround();
	controller->BuildPoliState(state);
	// std::cout << state.transpose() << std::endl;

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;

}

std::vector<double> cSimAdapter::getLLCState()
{

	Eigen::VectorXd state;
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	auto controller = std::dynamic_pointer_cast<cWaypointController>(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();
		llc->BuildPoliState(state);
	}

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;
}

std::vector<double> cSimAdapter::getImitationState() const
{
	// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
	std::shared_ptr<cScenarioImitateVizEval> scenario__tmp = std::dynamic_pointer_cast<cScenarioImitateVizEval>(this->_scene);
	if ( scenario__tmp != nullptr)
	{
		Eigen::VectorXd state;
		const std::shared_ptr<cKinSimCharacter> char_ = std::dynamic_pointer_cast<cKinSimCharacter>(scenario__tmp->GetKinChar());
		const std::shared_ptr<cSimCharacter> char__ = this->_scene->GetCharacter();
		std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char__->GetController());
		char_->BuildKinCharPoliState(state);
		std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
		std::string char_ctrl = "";
		gArgParser->ParseString("char_ctrl", char_ctrl);
		// std::cout << char_ctrl << std::endl;
		if (char_ctrl == "ct_pd_grf")
		{ // The imitation character does not include contact dynamics
			// Add garbage data to let state vectors align
			if ( controller->Is3D() )
			{ /// hard-coded hack for hands and feet EOF
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
			}
			else
			{
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
				out.push_back(0);
			}
		}
		return out;
	}
	else
	{
		throw std::runtime_error("This scenario does not support imitation information. " + std::string(__FILE__)+ ":"+ std::to_string(__LINE__));
	}

}

std::vector<double> cSimAdapter::getImitationStateAtTime(double animTime)
{
	// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
	std::shared_ptr<cScenarioImitateVizEval> scenario__tmp = std::dynamic_pointer_cast<cScenarioImitateVizEval>(this->_scene);
	if ( scenario__tmp != nullptr)
	{
		Eigen::VectorXd state;
		const std::shared_ptr<cKinSimCharacter> char_ = std::dynamic_pointer_cast<cKinSimCharacter>(scenario__tmp->GetKinChar());
		double before_animTime = char_->GetTime();
		char_->SetTime(animTime);
		char_->BuildKinCharPoliState(state);
		char_->SetTime(before_animTime);
		std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());

		return out;
	}
	else
	{
		throw std::runtime_error("This scenario does not support imitation information. " + std::string(__FILE__)+ ":"+ std::to_string(__LINE__));
	}

}

double cSimAdapter::getAnimationTime() const
{
	// std::shared_ptr<cScenarioImitate> scenario__ = std::shared_ptr<cScenarioImitate>(new cScenarioImitate());
	std::shared_ptr<cScenarioImitateVizEval> scenario__tmp = std::dynamic_pointer_cast<cScenarioImitateVizEval>(this->_scene);
	if ( scenario__tmp != nullptr)
	{
		Eigen::VectorXd state;
		const std::shared_ptr<cKinSimCharacter> char_ = std::dynamic_pointer_cast<cKinSimCharacter>(scenario__tmp->GetKinChar());
		return char_->GetTime();
	}
	else
	{
		throw std::runtime_error("This scenario does not support imitation information. " + std::string(__FILE__)+ ":"+ std::to_string(__LINE__));
	}
}

void cSimAdapter::setSimState(std::vector<double> state_)
{
	std::shared_ptr<cScenarioImitateVizEval> scenario__tmp = std::dynamic_pointer_cast<cScenarioImitateVizEval>(this->_scene);
	if ( scenario__tmp != nullptr)
	{
		const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

		// Eigen::VectorXd pose(state_.data());
		double* ptr = &state_[0];
		Eigen::Map<Eigen::VectorXd> pose(ptr, state_.size());


		Eigen::VectorXd pose0 = char_->GetPose();
		Eigen::VectorXd vel0 = char_->GetVel();
		char_->SetPose(pose.segment(0,pose0.rows() * pose0.cols()));
		char_->SetVel(pose.segment(pose0.rows() * pose0.cols(),vel0.rows() * vel0.cols()));

		const std::shared_ptr<cKinSimCharacter> kchar_ = std::dynamic_pointer_cast<cKinSimCharacter>(scenario__tmp->GetKinChar());
		kchar_->SetTime(pose( (pose.rows() * pose.cols()) - 1) );
	}
	else
	{
		throw std::runtime_error("This scenario does not support imitation information. " + std::string(__FILE__)+ ":"+ std::to_string(__LINE__));
	}
}

std::vector<double> cSimAdapter::getSimState() const
{
	double animPhase = getAnimationTime();
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	Eigen::VectorXd pose0 = char_->GetPose();
	Eigen::VectorXd vel0 = char_->GetVel();

	std::vector<double> out(pose0.data(), pose0.data() + pose0.rows() * pose0.cols());
	std::vector<double> tmp(vel0.data(), vel0.data() + vel0.rows() * vel0.cols());
	out.insert(out.end(),tmp.begin(),tmp.end());
	// out.extend(vel0.data());
	out.push_back(animPhase);
	return out;
}

std::vector<double> cSimAdapter::getJointWeights() const
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	int num_joints = char_->GetNumJoints();
	std::vector<double> out_weights = std::vector<double>(num_joints);
	double sum = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		double curr_w = char_->GetJointDiffWeight(j);
		out_weights[j] = curr_w;
		sum = sum + curr_w;
	}
	return out_weights;
}

bool cSimAdapter::endOfEpoch()
{

	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

	// std::cout << "Fallen: " << char_->HasFallen() << std::endl;
	if (char_->HasFallen() || char_->HasExploded() || this->_scene->endOfEpoch())
	{
		// std::cout << "End of Epoch:" << std::endl;
		return true;
	}
	return false;
}

bool cSimAdapter::agentHasFallen()
{
	// const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

	// this->_scene->HasFallen();

	return this->_scene->HasFallen() || this->_scene->endOfEpoch();
}


bool cSimAdapter::agent_contact(size_t agent_num)
{
	// const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();

	// this->_scene->HasFallen();
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	return agent->IsInContact_for_collision();


}



bool cSimAdapter::needUpdatedAction()
{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());

	int con_state_ = controller->GetState();
	// std::cout << "Controller State: " << con_state_ << " last controller state: "<< _lastControllerState << std::endl;
	if (con_state_ == 0 && (_lastControllerState != 0))
	{
		_lastControllerState = con_state_;
		/// This will update the sim to properly calculate the terrain input
		// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
		// this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
		return true;
	}
	_lastControllerState = con_state_;
	return false; // 0 = start of new action
}

void cSimAdapter::onKeyEvent(int key, int mouseX, int mouseY)
{
	/*
	std::shared_ptr<cDrawScenario> scenario__ = std::static_pointer_cast<cDrawScenario>(this->_gScenario);
	if (scenario__ != NULL)
	{
		scenario__->Keyboard(key, mouseX, mouseY);
	}
	*/
	Keyboard(key, mouseX, mouseY);
}

void cSimAdapter::setRelativeFilePath(std::string relativePath)
{
	this->_relativePath = relativePath;
}

void cSimAdapter::setRandomSeed(int seed)
{
	this->_scene->SetRandSeed(seed);
}

/*
 * New stuff for multi character simulation
 */

size_t cSimAdapter::getNumAgents()
{
	std::shared_ptr<cScenarioSpaceMultChar> scenario__tmp = std::dynamic_pointer_cast<cScenarioSpaceMultChar>(this->_scene);
	if (scenario__tmp != nullptr)
	{
		return scenario__tmp->GetCharacterCount() + 1;  // 1 for default character
	}

	std::shared_ptr<cScenarioMultChar> scenario__tmp2 = std::dynamic_pointer_cast<cScenarioMultChar>(this->_scene);
	if (scenario__tmp2 != nullptr)
	{
		return scenario__tmp2->GetCharacterCount() + 1;  // 1 for default character
	}

	return 1;
}

size_t cSimAdapter::getActionSpaceSize() const
{
	if ( this->_scene != nullptr )
	{
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(char_->GetController());
	return controller->GetPoliActionSize();
	}
	return 0;
}

std::vector<std::vector<double> > cSimAdapter::getActionSpaceBounds() const
{
	std::vector<std::vector<double> > bounds;
	if ( this->_scene != nullptr )
	{

		const std::shared_ptr<cSimCharacter> agent = this->_scene->GetCharacter();

		std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(agent->GetController());

		Eigen::VectorXd out_min;
		Eigen::VectorXd out_max;
		controller->GetPoliActionBounds(out_min, out_max);

		std::vector<double> min(out_min.data(), out_min.data() + out_min.rows() * out_min.cols());
		std::vector<double> max(out_max.data(), out_max.data() + out_max.rows() * out_max.cols());

		bounds.push_back(min);
		bounds.push_back(max);

	}
	return bounds;
}

size_t cSimAdapter::getObservationSpaceSize() const
{

	return this->getState().size();
}


std::shared_ptr<cSimCharacter> cSimAdapter::getAgent(size_t agent_num)
{
	if ( agent_num == 0 )
	{
		return this->_scene->GetCharacter();
	}
	else
	{
		std::shared_ptr<cScenarioSpaceMultChar> scenario__tmp = std::dynamic_pointer_cast<cScenarioSpaceMultChar>(this->_scene);
		if (scenario__tmp != nullptr)
		{
			return scenario__tmp->mChars[agent_num - 1];
		}
		std::shared_ptr<cScenarioMultChar> scenario__tmp2 = std::dynamic_pointer_cast<cScenarioMultChar>(this->_scene);
		if (scenario__tmp2 != nullptr)
		{
			return scenario__tmp2->mChars[agent_num - 1];
		}

	}
}

double cSimAdapter::updateActionForAgent(size_t agent_num, std::vector<double> act)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);

	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(agent->GetController());
	// std::cout << "Controller Action size: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	cTerrainRLCharController::tAction action;
	action.mID;
	action.mParams = Eigen::VectorXd::Zero(act.size());
	for (size_t i=0; i < act.size(); i++)
	{
		action.mParams(i) = act[i];
	}

	controller->ApplyAction(action);
	return 0.0;



}

double cSimAdapter::updateLLCActionForAgent(size_t agent_num, std::vector<double> act)
{
	const std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	// char_ = scene->getChar();
	auto controller = std::dynamic_pointer_cast<cWaypointController>(agent->GetController());
		// std::shared_ptr<cWaypointController> controller =  std::static_pointer_cast< cWaypointController >(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();

		cTerrainRLCharController::tAction action;
		action.mID;
		action.mParams = Eigen::VectorXd::Zero(act.size());
		for (size_t i=0; i < act.size(); i++)
		{
			action.mParams(i) = act[i];
		}

		llc->ApplyAction(action);
	}
	// std::cout << "Action length: " << controller->GetPoliActionSize() << std::endl;
	// std::cout << "Num controller params: " << controller->GetNumParams() << std::endl;
	// ASSERT(controller->GetPoliActionSize() == act.size(),  "Action length (" << act.size() << ") incorrect, should be " << controller->GetPoliActionSize());

	return 0.0;

}

/// Returns the reward for executing action act
void cSimAdapter::actForAgent(std::vector<double> act)
{
	//TODO: does not function for not, outdated
}

double cSimAdapter::calcRewardForAgent(size_t agent_num)
{
	double reward = 0.0;
	//TODO: Rewards are not agent specific yet
	std::shared_ptr<cScenarioSpaceMultChar> scenario__tmp = std::dynamic_pointer_cast<cScenarioSpaceMultChar>(this->_scene);
	std::shared_ptr<cScenarioMultChar> scenario__tmp2 = std::dynamic_pointer_cast<cScenarioMultChar>(this->_scene);
	if (scenario__tmp != nullptr)
	{
		if ( agent_num == 0 )
		{
			reward = scenario__tmp->CalcReward();
		}
		else
		{
			reward = scenario__tmp->calcRewardForAgent(agent_num - 1);
		}
	}
	else if ( scenario__tmp2 != nullptr )
	{
		if ( agent_num == 0 )
		{
			reward = scenario__tmp2->CalcReward();
		}
		else
		{
			reward = scenario__tmp2->calcRewardForAgent(agent_num - 1);
		}
	}
	else
	{
		reward = this->calcReward();
	}


	return reward;
}

double cSimAdapter::calcVelocity(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	return agent->GetRootVel()(0);
}

bool cSimAdapter::hasStumbledForAgent(size_t agent_num)
{

	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	return this->hasStumbled();
}

double cSimAdapter::jointTorque(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	return 0.0;
}

bool cSimAdapter::endOfEpochForAgent(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);

	Eigen::VectorXd pose0 = agent->GetPose();
	// std::cout << "agent " << agent_num << " Fallen: " << agent->HasFallen() << " Exploded?: " << agent->HasExploded() << std::endl;
	if (agent->HasFallen() || agent->HasExploded() ||
			(pose0[1] < -10.0) || this->_scene->endOfEpoch())
	{
		// std::cout << "End of Epoch:" << std::endl;
		return true;
	}
	return false;
}

bool cSimAdapter::agentHasFallenForAgent(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	return agent->HasFallen();
}

bool cSimAdapter::needUpdatedActionForAgent(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);

	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(agent->GetController());

	int con_state_ = controller->GetState();
	// std::cout << "Controller State: " << con_state_ << " last controller state: "<< _lastControllerState << std::endl;
	if (con_state_ == 0 && (_lastControllerState != 0))
	{
		_lastControllerState = con_state_;
		/// This will update the sim to properly calculate the terrain input
		// std::shared_ptr<cDrawScenarioSimChar> scenario__ = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
		// this->_scene = std::shared_ptr<cScenarioSimChar>(scenario__->GetScene());
		return true;
	}
	_lastControllerState = con_state_;
	return false; // 0 = start of new action

}

std::vector<double> cSimAdapter::getStateForAgent(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);

	Eigen::VectorXd state;
	// char_ = scene->getChar();
	std::shared_ptr<cTerrainRLCharController> controller =  std::static_pointer_cast< cTerrainRLCharController >(agent->GetController());
	// const std::shared_ptr<cTerrainRLCharController>& controller =  static_cast< const std::shared_ptr<cTerrainRLCharController>& >(char_->GetController());
	// controller = char->getCOntroller()
	controller->ParseGround();
	controller->BuildPoliState(state);
	// std::cout << state.transpose() << std::endl;

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;

}

std::vector<double> cSimAdapter::getLLCStateForAgent(size_t agent_num)
{
	std::shared_ptr<cSimCharacter> agent = this->getAgent(agent_num);
	Eigen::VectorXd state;
	auto controller = std::dynamic_pointer_cast<cWaypointController>(agent->GetController());
	// std::shared_ptr<cWaypointController> controller =  std::static_pointer_cast< cWaypointController >(char_->GetController());
	if (controller != nullptr)
	{
		std::shared_ptr<cBipedStepController3D> llc = controller->GetLLC();
		llc->BuildPoliState(state);
	}
	// const std::shared_ptr<cTerrainRLCharController>& controller =  static_cast< const std::shared_ptr<cTerrainRLCharController>& >(char_->GetController());
	// controller = char->getCOntroller()
	// controller->ParseGround();
	// controller->BuildPoliState(state);
	// std::cout << state.transpose() << std::endl;

	std::vector<double> out(state.data(), state.data() + state.rows() * state.cols());
	return out;

}

void cSimAdapter::handleUpdatedAction()
{
	auto sc = std::dynamic_pointer_cast<cScenarioExp>(this->_scene);
	if ( sc != nullptr )
	{
		sc->HandleNewActionUpdate();
	}
}

size_t cSimAdapter::getTaskID() const
{
	auto sc = std::dynamic_pointer_cast<cScenarioMultiTaskImitateVizEval>(this->_scene);
	if ( sc != nullptr )
	{
		return sc->getMotionID() + 1;
	}

	auto sc2 = std::dynamic_pointer_cast<cScenarioImitateEvalMultiTask>(this->_scene);
	if ( sc2 != nullptr )
	{
		 return sc2->getTaskID();
	}

	return 0;
}

void cSimAdapter::setTaskID(size_t task)
{
	auto sc = std::dynamic_pointer_cast<cScenarioMultiTaskImitateVizEval>(this->_scene);
	if ( sc != nullptr )
	{
		 sc->setMotionID(task);
	}

	auto sc2 = std::dynamic_pointer_cast<cScenarioImitateEvalMultiTask>(this->_scene);
	if ( sc2 != nullptr )
	{
		 return sc2->setTaskID(task);
	}

}

size_t cSimAdapter::GetNumTasks() const
{
	auto sc = std::dynamic_pointer_cast<cScenarioMultiTaskImitateVizEval>(this->_scene);
	if ( sc != nullptr )
	{
		 return sc->GetNumMotions();
	}

	auto sc2 = std::dynamic_pointer_cast<cScenarioImitateEvalMultiTask>(this->_scene);
	if ( sc2 != nullptr )
	{
		 return sc2->GetNumTasks();
	}

	return 1;

}
