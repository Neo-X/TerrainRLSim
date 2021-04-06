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

	light_dir = TinyRender::Vec3f(1,1,1); // light source
	eye = TinyRender::Vec3f(1,1,3); // camera position
	center = TinyRender::Vec3f(0,0,0); // camera direction
	up = TinyRender::Vec3f(0,1,0); // camera up vector

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
	TinyRender::lookat(eye, center, up);                            // build the ModelView matrix
	TinyRender::viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
	TinyRender::projection(-1.f/(eye-center).norm());               // build the Projection matrix

//	for (int m=1; m<argc; m++) { // iterate through all input objects
//		Model model(argv[m]);
//		Shader shader(model);
	const std::shared_ptr<cSimCharacter> char_ = this->scenario->GetCharacter();
//	char_->
	const auto& shape_defs = char_->GetDrawShapeDefs();
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

	const tVector pos = tVector(0,0,0,0);
	const tVector size = tVector(1,2,3,0);
	const tVector tex_coord_min = tVector::Zero();
	const tVector tex_coord_max = tVector::Ones();

	const int num_faces = 6;
	const int pos_len = num_faces * 6 * 3;
	const int coord_len = num_faces * 6 * 3;


	tVector sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
	tVector nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);

	tVector sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	tVector nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);


	if( size[1]>1)
	{
		sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.0 * size[1]  , pos[2] - 0.5 * size[2], 0);
		se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.0 * size[1] , pos[2] - 0.5 * size[2], 0);
		ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);
		nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], 0);

		sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.0 * size[1], pos[2] + 0.5 * size[2], 0);
		se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.0 * size[1], pos[2] + 0.5 * size[2], 0);
		ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
		nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], 0);
	}


	const float pos_data[pos_len] = {
		ne0[0], ne0[1], ne0[2], // top
		nw0[0], nw0[1], nw0[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		ne1[0], ne1[1], ne1[2],
		ne0[0], ne0[1], ne0[2],

		se1[0], se1[1], se1[2],  // bottom
		sw1[0], sw1[1], sw1[2],
		sw0[0], sw0[1], sw0[2],
		sw0[0], sw0[1], sw0[2],
		se0[0], se0[1], se0[2],
		se1[0], se1[1], se1[2],

		se1[0], se1[1], se1[2], // front
		se0[0], se0[1], se0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		ne1[0], ne1[1], ne1[2],
		se1[0], se1[1], se1[2],

		sw0[0], sw0[1], sw0[2], // back
		sw1[0], sw1[1], sw1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		nw0[0], nw0[1], nw0[2],
		sw0[0], sw0[1], sw0[2],

		sw0[0], sw0[1], sw0[2], // left
		nw0[0], nw0[1], nw0[2],
		ne0[0], ne0[1], ne0[2],
		ne0[0], ne0[1], ne0[2],
		se0[0], se0[1], se0[2],
		sw0[0], sw0[1], sw0[2],

		se1[0], se1[1], se1[2], // right
		ne1[0], ne1[1], ne1[2],
		nw1[0], nw1[1], nw1[2],
		nw1[0], nw1[1], nw1[2],
		sw1[0], sw1[1], sw1[2],
		se1[0], se1[1], se1[2]
	};

	const float coord_data[coord_len] = {
		tex_coord_min[0], tex_coord_min[2], // top
		tex_coord_max[0], tex_coord_min[2],
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_min[2],

		tex_coord_max[0], tex_coord_min[2], // bottom
		tex_coord_max[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_max[2],
		tex_coord_min[0], tex_coord_min[2],
		tex_coord_max[0], tex_coord_min[2],

		tex_coord_min[0], tex_coord_min[1], // front
		tex_coord_max[0], tex_coord_min[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_min[1],

		tex_coord_min[0], tex_coord_min[1], // back
		tex_coord_max[0], tex_coord_min[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_max[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_max[1],
		tex_coord_min[0], tex_coord_min[1],

		tex_coord_max[2], tex_coord_min[1], // left
		tex_coord_max[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_min[1],
		tex_coord_max[2], tex_coord_min[1],

		tex_coord_max[2], tex_coord_min[1], // right
		tex_coord_max[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_max[1],
		tex_coord_min[2], tex_coord_min[1],
		tex_coord_max[2], tex_coord_min[1]
	};


//	tAttribInfo attr_info;
//	attr_info.mAttribNumber = cMeshUtil::eAttributePosition;
//	attr_info.mAttribSize = sizeof(pos_data[0]);
//	attr_info.mDataOffset = 0;
//	attr_info.mDataStride = 0;
//	attr_info.mNumComp = cMeshUtil::gPosDim;
//	gBoxSolidMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * pos_len, (GLubyte*)pos_data, 0, 1, &attr_info);
//
//	attr_info.mAttribNumber = cMeshUtil::eAttributeCoord;
//	attr_info.mAttribSize = sizeof(coord_data[0]);
//	attr_info.mDataOffset = 0;
//	attr_info.mDataStride = 0;
//	attr_info.mNumComp = cMeshUtil::gCoordDim;
//	gBoxSolidMesh->LoadVBuffer(attr_info.mAttribNumber, sizeof(float) * coord_len, (GLubyte*)coord_data, 0, 1, &attr_info);
//
//	setMatricesFromStack();
//	gBoxSolidMesh->Draw(gl_mode);
}


