/*
 * SimAdapter.cpp
 *
 *  Created on: 2021-04-03
 *      Author: Glen Berseth
 */

#include "TinyRendererWrapper.h"


cTinyRendererWrapper::cTinyRendererWrapper() {
	// TODO Auto-generated constructor stub

	width  = 800; // output image size
	height = 800;

	light_dir = vec3(1,1,1); // light source
	eye = vec3(1,1,3); // camera position
	center = vec3(0,0,0); // camera direction
	up = vec3(0,1,0); // camera up vector

//	extern mat<4,4> ModelView; // "OpenGL" state matrices
//	extern mat<4,4> Projection;

}

cTinyRendererWrapper::~cTinyRendererWrapper() {
	// TODO Auto-generated destructor stub
}


void cTinyRendererWrapper::init()
{
	std::vector<double> zbuffer(width*height, -std::numeric_limits<double>::max()); // note that the z-buffer is initialized with minimal possible values
	TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
	lookat(eye, center, up);                            // build the ModelView matrix
	viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
	projection(-1.f/(eye-center).norm());               // build the Projection matrix

//	for (int m=1; m<argc; m++) { // iterate through all input objects
//		Model model(argv[m]);
//		Shader shader(model);
//		for (int i=0; i<model.nfaces(); i++) { // for every triangle
//			vec4 clip_vert[3]; // triangle coordinates (clip coordinates), written by VS, read by FS
//			for (int j=0; j<3; j++)
//				clip_vert[j] = shader.vertex(i, j); // call the vertex shader for each triangle vertex
//			triangle(clip_vert, shader, framebuffer, zbuffer); // actual rasterization routine call
//		}
//	}
	framebuffer.write_tga_file("framebuffer.tga"); // the vertical flip is moved inside the function
//	return 0;
}

