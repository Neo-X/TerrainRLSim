/*
 * SimAdapter.cpp
 *
 *  Created on: 2021-04-03
 *      Author: Glen Berseth
 */

#include "TinyRendererWrapper.h"
#include <memory>
#include "tinyrenderer/geometry.h"

using namespace TinyRender;

//struct Shader : public IShader
//{
//	Model* m_model;
//	Vec3f m_light_dir_local;
//	Vec3f m_light_color;
//	Matrix& m_modelMat;
//	Matrix m_invModelMat;
//	Matrix& m_modelView1;
//	Matrix& m_projectionMat;
//	Vec3f m_localScaling;
//	Matrix& m_lightModelView;
//	Vec4f m_colorRGBA;
//	Matrix& m_viewportMat;
//	Matrix m_projectionModelViewMat;
//	Matrix m_projectionLightViewMat;
//	float m_ambient_coefficient;
//	float m_diffuse_coefficient;
//	float m_specular_coefficient;
//
//	std::vector<float>* m_shadowBuffer;
//
//	int m_width;
//	int m_height;
//
//	int m_index;
//
//	mat<2, 3, float> varying_uv;   // triangle uv coordinates, written by the vertex shader, read by the fragment shader
//	mat<4, 3, float> varying_tri;  // triangle coordinates (clip coordinates), written by VS, read by FS
//	mat<4, 3, float> varying_tri_light_view;
//	mat<3, 3, float> varying_nrm;  // normal per vertex to be interpolated by FS
//	mat<4, 3, float> world_tri;    // model triangle coordinates in the world space used for backface culling, written by VS
//
//	Shader(Model* model, Vec3f light_dir_local, Vec3f light_color, Matrix& modelView, Matrix& lightModelView, Matrix& projectionMat, Matrix& modelMat, Matrix& viewportMat, Vec3f localScaling, const Vec4f& colorRGBA, int width, int height, std::vector<float>* shadowBuffer, float ambient_coefficient = 0.6, float diffuse_coefficient = 0.35, float specular_coefficient = 0.05)
//		: m_model(model),
//		  m_light_dir_local(light_dir_local),
//		  m_light_color(light_color),
//		  m_modelMat(modelMat),
//		  m_modelView1(modelView),
//		  m_projectionMat(projectionMat),
//		  m_localScaling(localScaling),
//		  m_lightModelView(lightModelView),
//		  m_colorRGBA(colorRGBA),
//		  m_viewportMat(viewportMat),
//		  m_ambient_coefficient(ambient_coefficient),
//		  m_diffuse_coefficient(diffuse_coefficient),
//		  m_specular_coefficient(specular_coefficient),
//
//		  m_shadowBuffer(shadowBuffer),
//		  m_width(width),
//		  m_height(height)
//
//	{
//		m_nearPlane = m_projectionMat.col(3)[2] / (m_projectionMat.col(2)[2] - 1);
//		m_farPlane = m_projectionMat.col(3)[2] / (m_projectionMat.col(2)[2] + 1);
//		//printf("near=%f, far=%f\n", m_nearPlane, m_farPlane);
//		m_invModelMat = m_modelMat.invert_transpose();
//		m_projectionModelViewMat = m_projectionMat * m_modelView1;
//		m_projectionLightViewMat = m_projectionMat * m_lightModelView;
//	}
//	virtual Vec4f vertex(int iface, int nthvert)
//	{
//		//B3_PROFILE("vertex");
//		Vec2f uv = m_model->uv(iface, nthvert);
//		varying_uv.set_col(nthvert, uv);
//		varying_nrm.set_col(nthvert, proj<3>(m_invModelMat * embed<4>(m_model->normal(iface, nthvert), 0.f)));
//		Vec3f unScaledVert = m_model->vert(iface, nthvert);
//		Vec3f scaledVert = Vec3f(unScaledVert[0] * m_localScaling[0],
//								 unScaledVert[1] * m_localScaling[1],
//								 unScaledVert[2] * m_localScaling[2]);
//		Vec4f gl_Vertex = m_projectionModelViewMat * embed<4>(scaledVert);
//		varying_tri.set_col(nthvert, gl_Vertex);
//		Vec4f world_Vertex = m_modelMat * embed<4>(scaledVert);
//		world_tri.set_col(nthvert, world_Vertex);
//		Vec4f gl_VertexLightView = m_projectionLightViewMat * embed<4>(scaledVert);
//		varying_tri_light_view.set_col(nthvert, gl_VertexLightView);
//		return gl_Vertex;
//	}
//
//	virtual bool fragment(Vec3f bar, TGAColor& color)
//	{
//		//B3_PROFILE("fragment");
//		Vec4f p = m_viewportMat * (varying_tri_light_view * bar);
//		float depth = p[2];
//		p = p / p[3];
//
//		float index_x = b3Max(float(0.0), b3Min(float(m_width - 1), p[0]));
//		float index_y = b3Max(float(0.0), b3Min(float(m_height - 1), p[1]));
//		int idx = int(index_x) + int(index_y) * m_width;                       // index in the shadowbuffer array
//		float shadow = 1.0;
//		if (m_shadowBuffer && idx >=0 && idx <m_shadowBuffer->size())
//		{
//			shadow = 0.8 + 0.2 * (m_shadowBuffer->at(idx) < -depth + 0.05);  // magic coeff to avoid z-fighting
//		}
//		Vec3f bn = (varying_nrm * bar).normalize();
//		Vec2f uv = varying_uv * bar;
//
//		Vec3f reflection_direction = (bn * (bn * m_light_dir_local * 2.f) - m_light_dir_local).normalize();
//        float specular = std::pow(b3Max(reflection_direction.z, 0.f),
//                                    m_model->specular(uv));
//        float diffuse = b3Max(0.f, bn * m_light_dir_local);
//
//        color = m_model->diffuse(uv);
//		color[0] *= m_colorRGBA[0];
//		color[1] *= m_colorRGBA[1];
//		color[2] *= m_colorRGBA[2];
//		color[3] *= m_colorRGBA[3];
//
//		for (int i = 0; i < 3; ++i)
//		{
//			int orgColor = 0;
//			float floatColor = (m_ambient_coefficient * color[i] + shadow * (m_diffuse_coefficient * diffuse + m_specular_coefficient * specular) * color[i] * m_light_color[i]);
//			if (floatColor==floatColor)
//			{
//				orgColor=int(floatColor);
//			}
//			color[i] = b3Min(orgColor, 255);
//		}
//
//		return false;
//	}
//};

cTinyRendererWrapper::cTinyRendererWrapper() {
	// TODO Auto-generated constructor stub

	width  = 800; // output image size
	height = 800;

	light_dir = TinyRender::Vec3f(1,1,1); // light source
	eye = TinyRender::Vec3f(2,2,5); // camera position
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
//	addBoxToScene();
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

	const std::shared_ptr<cSimCharacter> char_ = this->scenario->GetCharacter();
	tVector c_pos = char_->CalcCOM();
	const auto& shape_defs = char_->GetDrawShapeDefs();
	size_t num_shapes = shape_defs.rows();
	eye = TinyRender::Vec3f(1,1,5) + TinyRender::Vec3f(c_pos[0], c_pos[1], c_pos[2]); // camera position
	center = TinyRender::Vec3f(c_pos[0], c_pos[1], c_pos[2]); // camera direction

	std::vector<float> zbuffer(width*height, -std::numeric_limits<double>::max()); // note that the z-buffer is initialized with minimal possible values
	TGAImage framebuffer(width, height, TGAImage::RGB); // the output image
	framebuffer.clear();
	ModelView = TinyRender::lookat(eye, center, up);                            // build the ModelView matrix
	Matrix viewPort = TinyRender::viewport(width/8, height/8, width*3/4, height*3/4); // build the Viewport matrix
	Projection = TinyRender::projection(-1.f/(eye-center).norm());               // build the Projection matrix

//	for (int m=1; m<argc; m++) { // iterate through all input objects
//		Model model(argv[m]);
//		Shader shader(model);

//	cDrawUtil::SetLineWidth(1);
//	for (int i = 0; i < num_shapes; ++i)
//	{
//	cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
//	}
//	Model model(argv[m]);
//	Shader shader(m_model);
//	Vec3f light_dir_local = Vec3f(light_dir[0], light_dir[1], light_dir[2]);
//	Vec3f light_color = Vec3f(1, 1, 1);
//	float light_distance = 20;
//	// light target is set to be the origin, and the up direction is set to be vertical up.
//	Matrix lightViewMatrix = lookat(light_dir_local * light_distance, Vec3f(0.0, 0.0, 0.0), Vec3f(0.0, 0.0, 1.0));
//	Matrix lightModelViewMatrix = ModelView;
//	Matrix modelViewMatrix = ModelView;
//	Vec3f localScaling(1, 1, 1);
//	Matrix viewMatrixInv = ModelView.invert();
//	btVector3 P(viewMatrixInv[0][3], viewMatrixInv[1][3], viewMatrixInv[2][3]);
//	float* shadowBufferPtr = 0;
	addBoxToScene();

//	Shader shader(m_model, light_dir_local, light_color, modelViewMatrix, lightModelViewMatrix, Projection, lightViewMatrix, lightViewMatrix, localScaling, m_model->getColorRGBA(), width, height, shadowBufferPtr, 1, 1, 1);
	for (size_t m = 0; m < m_models.size(); m++)
	{
		Shader shader(m_models[m], ModelView, Projection, light_dir);
		std::cout << "Rendering: verts: " << m_models[m].nverts() << " faces: " << m_models[m].nfaces() << std::endl;
		for (int i=0; i<m_models[m].nfaces(); i++) { // for every triangle
			TinyRender::mat<4, 4, float> clip_vert; // triangle coordinates (clip coordinates), written by VS, read by FS
			for (int j=0; j<3; j++)
			{
	//			std::cout << "face: " << i << " vert: " << j << std::endl;
	//			clip_vert[j] = proj<3>(shader.vertex(i, j)); // call the vertex shader for each triangle vertex
				clip_vert[j] = (shader.vertex(i, j)); // call the vertex shader for each triangle vertex
			}
			triangle2(clip_vert, shader, framebuffer, zbuffer, viewPort); // actual rasterization routine call
		}
	}
//	}
//	framebuffer.write_tga_file("framebuffer.tga"); // the vertical flip is moved inside the function
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
	m_models.clear();
	tVector pos = tVector(0,0,0,0);
	const tVector tex_coord_min = tVector::Zero();
	const tVector tex_coord_max = tVector::Ones();

	const int num_faces = 6;
	const int pos_len = num_faces * 6 * 3;
	const int coord_len = num_faces * 6 * 3;

	const std::shared_ptr<cSimCharacter> char_ = this->scenario->GetCharacter();
	const auto& shape_defs = char_->GetDrawShapeDefs();
	size_t num_shapes = shape_defs.rows();
	for (size_t d=0; d < shape_defs.rows(); d++)
	{
		cKinTree::tDrawShapeDef curr_def = shape_defs.row(d);

		double theta = 0;
		tVector euler = cKinTree::GetDrawShapeAttachTheta(curr_def);
		int parent_joint = cKinTree::GetDrawShapeParentJoint(curr_def);
		tVector attach_pt = cKinTree::GetDrawShapeAttachPt(curr_def);
		tVector col = cKinTree::GetDrawShapeColor(curr_def);
		tVector size = tVector(curr_def[cKinTree::eDrawShapeParam0],
				curr_def[cKinTree::eDrawShapeParam1], curr_def[cKinTree::eDrawShapeParam2], 0);
	//	col = col.cwiseProduct(fill_tint);

		tMatrix world_trans = char_->BuildJointWorldTrans(parent_joint);
		std::cout << "world_trans: " << world_trans << std::endl;
		std::cout << "col: " << col << std::endl;

	//	cDrawUtil::PushMatrix();
	//	cDrawUtil::GLMultMatrix(world_trans);
	//	cDrawUtil::Translate(attach_pt);
	//	cDrawUtil::Rotate(euler);

	//	cDrawUtil::SetColor(col);

	//	const tVector size = tVector(x,y,z,0);
		pos = pos;


	//	double theta;
		tVector axis;
		cMathUtil::EulerToAxisAngle(euler, axis, theta);
		tMatrix rot_m = cMathUtil::RotateMat(axis, theta);
		tMatrix trans_m = cMathUtil::TranslateMat(attach_pt);


		tVector sw0 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], - 0.5 * size[1], - 0.5 * size[2], 1));
		tVector se0 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], - 0.5 * size[1], - 0.5 * size[2], 1));
		tVector ne0 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], + 0.5 * size[1], - 0.5 * size[2], 1));
		tVector nw0 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], + 0.5 * size[1], - 0.5 * size[2], 1));

		tVector sw1 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], - 0.5 * size[1], + 0.5 * size[2], 1));
		tVector se1 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], - 0.5 * size[1], + 0.5 * size[2], 1));
		tVector ne1 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], + 0.5 * size[1], + 0.5 * size[2], 1));
		tVector nw1 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], + 0.5 * size[1], + 0.5 * size[2], 1));


		if( size[1]>1)
		{
			sw0 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], - 0.0 * size[1], - 0.5 * size[2], 1));
			se0 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], - 0.0 * size[1], - 0.5 * size[2], 1));
			ne0 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], + 0.5 * size[1], - 0.5 * size[2], 1));
			nw0 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], + 0.5 * size[1], - 0.5 * size[2], 1));

			sw1 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], - 0.0 * size[1], + 0.5 * size[2], 1));
			se1 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], - 0.0 * size[1], + 0.5 * size[2], 1));
			ne1 = pos + (world_trans * trans_m * rot_m * tVector(+ 0.5 * size[0], + 0.5 * size[1], + 0.5 * size[2], 1));
			nw1 = pos + (world_trans * trans_m * rot_m * tVector(- 0.5 * size[0], + 0.5 * size[1], + 0.5 * size[2], 1));
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


		TinyRender::Model model = TinyRender::Model();
		float rgbaColor[4] = {col[0],col[1],col[2],col[3]};
//		float rgbaColor[4] = {1,0,1,0};
		model.setColorRGBA(rgbaColor);


		model.reserveMemory(pos_len/3, pos_len/9);

		for (int i = 0; i < pos_len; i+=3)
		{
			model.addVertex(pos_data[i],
					pos_data[i + 1],
					pos_data[i + 2],
					0.0, //normal
					0.0,
					1.0,
					0.45, // uv
					0.55);
		}
		for (int i = 0; i < model.nverts(); i += 3)
		{
			model.addTriangle(i, i, i,
					i + 1, i + 1, i + 1,
					i + 2, i + 2, i + 2); // They are given in order
		}
		std::cout << "verts: " << model.nverts() << " faces: " << model.nfaces() << std::endl;
		m_models.push_back(model);
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
}


