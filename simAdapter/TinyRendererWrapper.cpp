/*
 * SimAdapter.cpp
 *
 *  Created on: 2021-04-03
 *      Author: Glen Berseth
 */

#include "TinyRendererWrapper.h"
#include <memory>



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
//	zbuffer(width*height, -std::numeric_limits<double>::max()); // note that the z-buffer is initialized with minimal possible values
//	framebuffer(width, height, TGAImage::RGB); // the output image

}

cTinyRendererWrapper::~cTinyRendererWrapper() {
	// TODO Auto-generated destructor stub
}


void cTinyRendererWrapper::init()
{
}

void cTinyRendererWrapper::setScene(std::shared_ptr<cScenarioSimChar> scene)
{
	this->scenario = scene;
}

//void cTinyRendererWrapper::render(vec3 light_dir, vec3 eye, vec3 center, vec3 up)
void cTinyRendererWrapper::render()
{


}

std::vector<unsigned char> cTinyRendererWrapper::getPixels()
{
	std::vector<double> zbuffer(width*height, -std::numeric_limits<double>::max()); // note that the z-buffer is initialized with minimal possible values
	TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
	lookat(eye, center, up);                            // build the ModelView matrix
	viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
	projection(-1.f/(eye-center).norm());               // build the Projection matrix

//	for (int m=1; m<argc; m++) { // iterate through all input objects
//		Model model(argv[m]);
//		Shader shader(model);
	const std::shared_ptr<cSimCharacter> char_ = this->_scene->GetCharacter();
	char_->
	const auto& shape_defs = character.GetDrawShapeDefs();
	size_t num_shapes = shape_defs.rows();

//	cDrawUtil::SetLineWidth(1);
//	for (int i = 0; i < num_shapes; ++i)
//	{
	cKinTree::tDrawShapeDef curr_def = shape_defs.row(0);
//	cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
//	}
//	for (int i=0; i<model.nfaces(); i++) { // for every triangle
//		vec4 clip_vert[3]; // triangle coordinates (clip coordinates), written by VS, read by FS
//		for (int j=0; j<3; j++)
//			clip_vert[j] = shader.vertex(i, j); // call the vertex shader for each triangle vertex
//		triangle(clip_vert, shader, framebuffer, zbuffer); // actual rasterization routine call
//	}
//	}
	addBoxToScene();
	framebuffer.write_tga_file("framebuffer.tga"); // the vertical flip is moved inside the function
	std::vector<unsigned char> out;
	for (size_t i = 0; i < width * height * 3; i ++)
	{
		out.push_back(framebuffer.buffer()[i]);
	}
	return out;
//	return 0;

}


void cTinyRendererWrapper::addBoxToScene()
{

}


